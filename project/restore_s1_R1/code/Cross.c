/*********************************************************************************************************************
* 纯摄像头十字处理，参考思路见Cross.h。原始二值图始终不改写。
* 逐列跟踪横向白臂的上下黑白交界，靠纵坐标突变找L角点；出口另行搜边，避免支路污染基础巡线。
* 所有几何/白区验证在写边线数组前完成。上下角点齐全时连接；缺近端角点时拟合真实出口向下延伸。
********************************************************************************************************************/
#include "Cross.h"

extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];
extern int Left_Lost_Flag[MT9V03X_H], Right_Lost_Flag[MT9V03X_H];
extern volatile int Left_Line[MT9V03X_H], Right_Line[MT9V03X_H];
extern volatile int Mid_Line[MT9V03X_H], Road_Wide[MT9V03X_H];
extern volatile int Island_State;

#define CROSS_CANDIDATES (8)
#define CROSS_MIN_WIDTH (15)

typedef struct
{
    int x, y;
    float slope;                       // dx/dy，只拟合实际连续边线
    uint8 valid;
} Cross_Corner;

typedef struct
{
    Cross_Corner p[CROSS_CORNER_COUNT];
    int end;
} Cross_Observation;

static Cross_State cross_state;
static uint8 cross_confirm, cross_recovered, cross_inside, cross_lower_missing;
static uint16 cross_hold;
static int cross_run_len;
static int near_left, near_right;
static int previous_center, previous_width, previous_y;
static int upper_profile[MT9V03X_W], lower_profile[MT9V03X_W];
static Cross_Corner candidates[4][CROSS_CANDIDATES];
static int candidate_count[4];
static Cross_Observation observation;
static uint8 filled[MT9V03X_H];

static int IAbs(int v) { return v < 0 ? -v : v; }
static float FAbs(float v) { return v < 0.0f ? -v : v; }
static int Round(float v) { return (int)(v + (v >= 0.0f ? 0.5f : -0.5f)); }

static int Pair_Valid(int row)
{
    int width = Right_Line[row] - Left_Line[row];
    return !Left_Lost_Flag[row] && !Right_Lost_Flag[row] &&
           Left_Line[row] >= 3 && Right_Line[row] < MT9V03X_W - 3 &&
           width >= CROSS_MIN_WIDTH && width < MT9V03X_W - 4 * CROSS_SIDE_OFFSET;
}

static int Find_Near(int *l, int *r)
{
    int row, k;
    for(row = MT9V03X_H - 2; row >= MT9V03X_H * 2 / 3; row--)
    {
        for(k = 0; k < CROSS_VALID_BELOW; k++)
        {
            if(!Pair_Valid(row - k)) { break; }
            if(IAbs(Left_Line[row-k] - Left_Line[row]) > CROSS_FIT_SEARCH ||
               IAbs(Right_Line[row-k] - Right_Line[row]) > CROSS_FIT_SEARCH) { break; }
        }
        if(k == CROSS_VALID_BELOW)
        {
            *l = Left_Line[row]; *r = Right_Line[row];
            return 1;
        }
    }
    return 0;
}

// 参考位置随当前近端跑道移动。左右白臂允许在不同高度出现，支持斜入十字。
static int Find_WhiteArm(int l, int r)
{
    int row, wide_count = 0, row_sum = 0, left_count = 0, right_count = 0;
    int best = 0, seed = -1;
    int mid = (l + r) / 2;
    int outside_l = l - CROSS_SIDE_OFFSET;
    int outside_r = r + CROSS_SIDE_OFFSET;
    if(outside_l < 3) { outside_l = 3; }
    if(outside_r > MT9V03X_W - 4) { outside_r = MT9V03X_W - 4; }
    for(row = CROSS_TOP_MARGIN; row < MT9V03X_H - 1; row++)
    {
        int wl = image_two_value[row][outside_l] == IMG_WHITE;
        int wr = image_two_value[row][outside_r] == IMG_WHITE;
        if(image_two_value[row][mid] == IMG_WHITE)
        {
            left_count += wl; right_count += wr;
            if(wl || wr) { wide_count++; row_sum += row; }
            if(wide_count >= CROSS_LOST_MIN && left_count >= CROSS_VALID_ABOVE &&
               right_count >= CROSS_VALID_ABOVE && wide_count > best)
            {
                best = wide_count;
                seed = row_sum / wide_count;
            }
        }
        else { wide_count = row_sum = left_count = right_count = 0; }
    }
    return seed;
}

// 需要连续3个真实黑像素；人工顶边/侧边不算边界。逐像素扫，避免漏掉下角点尖部。
static void Scan_Column(int x, int seed, int *up, int *down)
{
    int row, blacks;
    *up = *down = -1;
    blacks = 0;
    for(row = seed - 1; row >= 2; row--)
    {
        blacks = (image_two_value[row][x] == IMG_BLACK) ? blacks + 1 : 0;
        if(blacks == 3) { *up = row + 3; break; }
    }
    blacks = 0;
    for(row = seed + 1; row < MT9V03X_H; row++)
    {
        blacks = (image_two_value[row][x] == IMG_BLACK) ? blacks + 1 : 0;
        if(blacks == 3) { *down = row - 3; break; }
    }
}

static void Build_Profiles(int center, int seed)
{
    int direction, x;
    for(x = 0; x < MT9V03X_W; x++) { upper_profile[x] = lower_profile[x] = -1; }
    for(direction = -1; direction <= 1; direction += 2)
    {
        int scan_y = seed;
        for(x = center; x >= 3 && x < MT9V03X_W - 3; x += direction)
        {
            int d, found = image_two_value[scan_y][x] == IMG_WHITE;
            // 沿横臂延续扫描高度，而非所有列固定从同一行起扫，兼容倾斜白臂。
            for(d = 1; !found && d <= MT9V03X_H / 4; d++)
            {
                if(scan_y-d >= CROSS_TOP_MARGIN && image_two_value[scan_y-d][x] == IMG_WHITE)
                { scan_y -= d; found = 1; }
                else if(scan_y+d < MT9V03X_H-1 && image_two_value[scan_y+d][x] == IMG_WHITE)
                { scan_y += d; found = 1; }
            }
            if(!found) { break; }
            Scan_Column(x, scan_y, &upper_profile[x], &lower_profile[x]);
            if(upper_profile[x] >= 0 && lower_profile[x] >= 0)
            {
                scan_y = (upper_profile[x] + lower_profile[x]) / 2;
            }
        }
    }
}

static int Edge_Supported(int row, int x, int left)
{
    int k, outside = left ? -1 : 1;
    for(k = 1; k <= 3; k++)
    {
        int c = x + outside * k;
        if(c < 2 || c >= MT9V03X_W-2 || image_two_value[row][c] != IMG_BLACK) { return 0; }
    }
    for(k = 0; k < 3; k++)
    {
        int c = x - outside * k;
        if(c < 2 || c >= MT9V03X_W-2 || image_two_value[row][c] != IMG_WHITE) { return 0; }
    }
    return 1;
}

static int Find_Side(int row, int previous, int left)
{
    int d, direction;
    for(d = 0; d <= CROSS_FIT_SEARCH; d++)
    {
        for(direction = -1; direction <= 1; direction += 2)
        {
            int x = previous + direction * d;
            if(x >= 3 && x < MT9V03X_W-3 && Edge_Supported(row, x, left)) { return x; }
        }
    }
    return -1;
}

static int Fit_Corner(int x, int y, int left, int upper, Cross_Corner *corner)
{
    int i, count = 0, previous = x;
    int sample_x[CROSS_FIT_ROWS], sample_dy[CROSS_FIT_ROWS];
    float sx = 0.0f, sy = 0.0f, sxy = 0.0f, syy = 0.0f, slope, intercept, residual = 0.0f;
    for(i = 0; i < CROSS_FIT_ROWS; i++)
    {
        int dy = (upper ? -1 : 1) * (i + 1);
        int row = y + dy;
        int found;
        if(row < 2 || row >= MT9V03X_H) { break; }
        found = Find_Side(row, previous, left);
        if(found < 0) { break; }
        sample_x[count] = found; sample_dy[count] = dy;
        sx += (float)found; sy += (float)dy;
        sxy += (float)(found * dy); syy += (float)(dy * dy);
        previous = found; count++;
    }
    if(count < (upper ? CROSS_VALID_ABOVE : CROSS_VALID_BELOW)) { return 0; }
    slope = ((float)count * sxy - sx * sy) / ((float)count * syy - sy * sy);
    intercept = (sx - slope * sy) / (float)count;
    if(FAbs(slope) > CROSS_MAX_DX_DY) { return 0; }
    for(i = 0; i < count; i++)
    {
        float error = (float)sample_x[i] - intercept - slope * (float)sample_dy[i];
        residual += error * error;
    }
    if(residual > (float)count * 2.5f) { return 0; }
    corner->x = Round(intercept); corner->y = y;
    corner->slope = slope; corner->valid = 1;
    return corner->x >= 3 && corner->x < MT9V03X_W-3;
}

static void Find_Corners(int which)
{
    int c, i;
    int upper = which < CROSS_LOWER_LEFT;
    int left = which == CROSS_UPPER_LEFT || which == CROSS_LOWER_LEFT;
    int inward = left ? 1 : -1;
    const int *profile = upper ? upper_profile : lower_profile;
    candidate_count[which] = 0;
    for(c = 3 + CROSS_PROFILE_SUPPORT; c < MT9V03X_W - 3 - CROSS_PROFILE_SUPPORT; c++)
    {
        int inside, jump, stable = 1;
        Cross_Corner corner;
        if(profile[c] < 0) { continue; }
        for(i = 1; i < CROSS_PROFILE_SUPPORT; i++)
        {
            int outward = c - inward * i;
            if(profile[outward] < 0 || IAbs(profile[outward] - profile[outward+inward]) > 2)
            { stable = 0; break; }
        }
        if(!stable) { continue; }
        inside = profile[c + inward * CROSS_PROFILE_SUPPORT];
        jump = upper ? profile[c] - inside : inside - profile[c];
        if(inside >= 0 && jump < CROSS_CORNER_JUMP) { continue; }
        if(!Fit_Corner(c + inward, profile[c] + (upper ? -1 : 1), left, upper, &corner)) { continue; }
        for(i = 0; i < candidate_count[which]; i++)
        {
            if(IAbs(corner.x - candidates[which][i].x) <= CROSS_PROFILE_SUPPORT * 2 &&
               IAbs(corner.y - candidates[which][i].y) <= CROSS_CORNER_JUMP) { break; }
        }
        if(i == candidate_count[which] && i < CROSS_CANDIDATES)
        {
            candidates[which][i] = corner;
            candidate_count[which]++;
        }
    }
}

static float Project(const Cross_Corner *corner, int row)
{
    return (float)corner->x + corner->slope * (float)(row - corner->y);
}

static float Guide(const Cross_Observation *obs, int side, int row)
{
    const Cross_Corner *up = &obs->p[side];
    const Cross_Corner *down = &obs->p[side + 2];
    if(down->valid && row >= up->y)
    {
        if(row > down->y) { return Project(down, row); }
        return (float)up->x + (float)(down->x - up->x) *
               (float)(row - up->y) / (float)(down->y - up->y);
    }
    return Project(up, row);
}

static Cross_Corner Choose_Lower(int which, const Cross_Corner *up, int near_x)
{
    Cross_Corner result;
    int i, best = MT9V03X_W * 4;
    result.valid = 0; result.x = result.y = 0; result.slope = 0.0f;
    for(i = 0; i < candidate_count[which]; i++)
    {
        const Cross_Corner *p = &candidates[which][i];
        int distance = IAbs(p->x - near_x);
        if(p->y - up->y < CROSS_LOST_MIN || distance > MT9V03X_W / 3) { continue; }
        if(FAbs((float)(p->x - up->x) / (float)(p->y - up->y)) > CROSS_MAX_DX_DY) { continue; }
        if(distance < best) { result = *p; best = distance; }
    }
    return result;
}

static int Validate_Observation(Cross_Observation *obs, int near_width)
{
    int row, col, i, white = 0, samples = 0;
    int top = obs->p[0].y < obs->p[1].y ? obs->p[0].y : obs->p[1].y;
    int far_row = top - CROSS_VALID_ABOVE;
    int bottom = MT9V03X_H - 2;
    if(far_row < 2) { return 0; }
    if(obs->p[2].valid && obs->p[3].valid)
    { bottom = obs->p[2].y > obs->p[3].y ? obs->p[2].y : obs->p[3].y; }
    if(bottom - top < CROSS_LOST_MIN) { return 0; }
    // 上角点外必须有一对真实出口边界，不能把路口横边/三岔黑块当成出口。
    for(i = 0; i < CROSS_VALID_ABOVE; i++)
    {
        int l = Round(Project(&obs->p[0], far_row-i));
        int r = Round(Project(&obs->p[1], far_row-i));
        if(far_row-i < 2) { break; }
        if(r-l < CROSS_MIN_WIDTH || r-l > near_width+4*CROSS_SIDE_OFFSET ||
           l < 3 || r >= MT9V03X_W-3 ||
           Find_Side(far_row-i, l, 1) < 0 || Find_Side(far_row-i, r, 0) < 0) { return 0; }
        for(col = 1; col <= 3; col++)
        {
            if(image_two_value[far_row-i][l+(r-l)*col/4] != IMG_WHITE) { return 0; }
        }
    }
    for(row = top+1; row <= bottom; row++)
    {
        int l = Round(Guide(obs, 0, row)), r = Round(Guide(obs, 1, row));
        if(l < 3 || r >= MT9V03X_W-3 || r-l < CROSS_MIN_WIDTH ||
           r-l > near_width+4*CROSS_SIDE_OFFSET) { return 0; }
        for(col = 1; col <= 3; col++)
        {
            white += image_two_value[row][l+(r-l)*col/4] == IMG_WHITE;
            samples++;
        }
    }
    obs->end = bottom;
    return white * 100 >= samples * CROSS_PATH_WHITE_PCT;
}

static int Observe(int seed, int l, int r)
{
    int i, j, which, best = MT9V03X_W * 10;
    int found = 0;
    Build_Profiles((l+r)/2, seed);
    for(which = 0; which < CROSS_CORNER_COUNT; which++) { Find_Corners(which); }
    for(i = 0; i < candidate_count[CROSS_UPPER_LEFT]; i++)
    {
        for(j = 0; j < candidate_count[CROSS_UPPER_RIGHT]; j++)
        {
            Cross_Observation candidate;
            int score;
            candidate.p[0] = candidates[CROSS_UPPER_LEFT][i];
            candidate.p[1] = candidates[CROSS_UPPER_RIGHT][j];
            if(candidate.p[1].x - candidate.p[0].x < CROSS_MIN_WIDTH) { continue; }
            candidate.p[2] = Choose_Lower(CROSS_LOWER_LEFT, &candidate.p[0], l);
            candidate.p[3] = Choose_Lower(CROSS_LOWER_RIGHT, &candidate.p[1], r);
            // 新识别至少要有一个入口角点；全白或只有远端缺口不能从NORMAL直接进入。
            if(!Cross_Flag && !candidate.p[2].valid && !candidate.p[3].valid) { continue; }
            if(!Validate_Observation(&candidate, r-l)) { continue; }
            score = IAbs(candidate.p[0].x + candidate.p[1].x - l - r);
            if(score < best) { observation = candidate; best = score; found = 1; }
        }
    }
    return found;
}

// 从新找到的上角点向远处重新搜真实边线，修复基础巡线可能跟到支路/左右归属错误的部分。
static void Trace_Exit(int side)
{
    int row, previous = observation.p[side].x;
    for(row = observation.p[side].y; row >= 2; row--)
    {
        int x = Find_Side(row, previous, side == 0);
        if(x < 0) { break; }
        if(side == 0) { Left_Line[row] = x; Left_Lost_Flag[row] = 0; }
        else { Right_Line[row] = x; Right_Lost_Flag[row] = 0; }
        previous = x;
    }
}

static void Fill_Observation(void)
{
    int row, side;
    for(side = 0; side < 2; side++)
    {
        const Cross_Corner *up = &observation.p[side];
        const Cross_Corner *down = &observation.p[side+2];
        int end = down->valid ? down->y : MT9V03X_H-1;
        Trace_Exit(side);
        for(row = up->y+1; row <= end; row++)
        {
            int x = Round(Guide(&observation, side, row));
            int center = Round((Guide(&observation, 0, row)+Guide(&observation, 1, row))*0.5f);
            if(x < 3 || x >= MT9V03X_W-3 || center < 3 || center >= MT9V03X_W-3) { continue; }
            // 只向当前白区补线，黑色遮挡不制造引导。
            if(image_two_value[row][center] != IMG_WHITE) { continue; }
            if(side == 0) { Left_Line[row] = x; Left_Lost_Flag[row] = 0; }
            else { Right_Line[row] = x; Right_Lost_Flag[row] = 0; }
            filled[row] |= (uint8)(1 << side);
        }
    }
    for(row = 2; row < MT9V03X_H; row++)
    {
        // 远处重新搜边若落到不同白块，不能把交叉/过窄的双边送进偏差计算。
        if(!Left_Lost_Flag[row] && !Right_Lost_Flag[row] && Right_Line[row]-Left_Line[row]<CROSS_MIN_WIDTH)
        { Left_Lost_Flag[row]=Right_Lost_Flag[row]=1; filled[row]=0; }
        Mid_Line[row] = (Left_Line[row]+Right_Line[row])/2;
        Road_Wide[row] = (!Left_Lost_Flag[row] && !Right_Lost_Flag[row]) ? Right_Line[row]-Left_Line[row] : 0;
    }
}

void Cross_Init(void)
{
    int i;
    Cross_Flag = 0; cross_state = CROSS_STATE_NORMAL;
    cross_confirm = cross_recovered = cross_inside = cross_lower_missing = 0;
    cross_hold = 0; cross_run_len = 0;
    near_left = MT9V03X_W/4; near_right = MT9V03X_W*3/4;
    previous_center = previous_width = previous_y = 0;
    for(i = 0; i < CROSS_CORNER_COUNT; i++) { observation.p[i].valid = 0; }
    for(i = 0; i < MT9V03X_H; i++) { filled[i] = 0; }
}

void Cross_Check(void)
{
    int row, run = 0, current_l, current_r, measured_l = 0, measured_r = 0, current_near, seed, observed = 0;
    if(Island_State != 0) { Cross_Init(); return; }
    cross_run_len = 0;
    for(row = 0; row < MT9V03X_H; row++)
    {
        filled[row] = 0;
        run = (Left_Lost_Flag[row] && Right_Lost_Flag[row]) ? run+1 : 0;
        if(run > cross_run_len) { cross_run_len = run; }
    }
    for(row = 0; row < CROSS_CORNER_COUNT; row++) { observation.p[row].valid = 0; }
    current_near = Find_Near(&measured_l, &measured_r);
    current_l = (Cross_Flag || !current_near) ? near_left : measured_l;
    current_r = (Cross_Flag || !current_near) ? near_right : measured_r;
    seed = Find_WhiteArm(current_l, current_r);
    if(seed >= 0) { observed = Observe(seed, current_l, current_r); }

    if(!Cross_Flag)
    {
        if(observed)
        {
            int center = (observation.p[0].x + observation.p[1].x)/2;
            int width = observation.p[1].x - observation.p[0].x;
            int y = (observation.p[0].y + observation.p[1].y)/2;
            if(cross_confirm && (IAbs(center-previous_center)>MT9V03X_W/6 ||
               IAbs(width-previous_width)>MT9V03X_W/6 || IAbs(y-previous_y)>MT9V03X_H/4))
            { cross_confirm = 0; }
            previous_center = center; previous_width = width; previous_y = y;
            if(cross_confirm < CROSS_CONFIRM_FRAMES) { cross_confirm++; }
            cross_state = CROSS_STATE_CANDIDATE;
            if(cross_confirm >= CROSS_CONFIRM_FRAMES)
            {
                Cross_Flag = 1; cross_state = CROSS_STATE_ENTER;
                cross_hold = CROSS_HOLD_FRAMES; cross_inside = cross_recovered = cross_lower_missing = 0;
                near_left = current_l; near_right = current_r;
                Fill_Observation();
            }
        }
        else
        {
            cross_confirm = 0; cross_state = CROSS_STATE_NORMAL;
            if(current_near)
            { near_left = current_l; near_right = current_r; }
        }
        return;
    }

    // 已在十字内也检查出口的帧间连续性，拒绝瞬间换到另一块白色支路。
    if(observed)
    {
        int center = (observation.p[0].x+observation.p[1].x)/2;
        int width = observation.p[1].x-observation.p[0].x;
        int y = (observation.p[0].y+observation.p[1].y)/2;
        if(IAbs(center-previous_center)>MT9V03X_W/6 || IAbs(width-previous_width)>MT9V03X_W/6 ||
           IAbs(y-previous_y)>MT9V03X_H/4)
        {
            observed = 0;
            for(row=0;row<CROSS_CORNER_COUNT;row++) { observation.p[row].valid=0; }
        }
        else { previous_center=center; previous_width=width; previous_y=y; }
    }
    if(observed)
    {
        int has_lower = observation.p[2].valid || observation.p[3].valid;
        cross_recovered = 0;
        // 保持计数只约束连续丢失可信出口的帧，不能把正常静止的路口图像当成超时。
        cross_hold = CROSS_HOLD_FRAMES;
        if(has_lower)
        {
            cross_lower_missing = 0;
            cross_state = cross_inside ? CROSS_STATE_EXIT : CROSS_STATE_ENTER;
        }
        else
        {
            if(cross_lower_missing < CROSS_PHASE_CONFIRM) { cross_lower_missing++; }
            if(cross_lower_missing >= CROSS_PHASE_CONFIRM) { cross_inside = 1; }
            cross_state = cross_inside ? CROSS_STATE_INSIDE : CROSS_STATE_ENTER;
        }
        Fill_Observation();
    }
    else
    {
        int recovered = 1;
        // 单帧角点漏检不是车头已进入内部的证据，保留上一个已确认阶段。
        cross_lower_missing = 0;
        if(cross_hold) { cross_hold--; }
        for(row = MT9V03X_H-2; row >= MT9V03X_H-1-CROSS_EXIT_ROWS; row--)
        {
            if(!Pair_Valid(row)) { recovered = 0; break; }
        }
        if(recovered && seed < 0)
        {
            cross_state = CROSS_STATE_EXIT;
            if(cross_recovered < CROSS_EXIT_FRAMES) { cross_recovered++; }
        }
        else { cross_recovered = 0; cross_state = cross_inside ? CROSS_STATE_INSIDE : CROSS_STATE_ENTER; }
    }
    if(!cross_hold || cross_recovered >= CROSS_EXIT_FRAMES)
    {
        Cross_Flag = 0; cross_state = CROSS_STATE_NORMAL;
        cross_confirm = cross_recovered = cross_inside = cross_lower_missing = 0;
        cross_hold = 0;
        // 本帧退出时未补线，Err不会把人工路宽学习为真实路宽。
        if(current_near)
        { near_left = measured_l; near_right = measured_r; }
    }
}

int Cross_GetRunLen(void) { return cross_run_len; }
Cross_State Cross_GetState(void) { return cross_state; }
uint8 Cross_GetCorner(int which, int *x, int *y)
{
    if(which < 0 || which >= CROSS_CORNER_COUNT || !x || !y || !observation.p[which].valid) { return 0; }
    *x = observation.p[which].x; *y = observation.p[which].y;
    return 1;
}
uint8 Cross_GetFillMask(int row)
{
    return (row >= 0 && row < MT9V03X_H) ? filled[row] : 0;
}
