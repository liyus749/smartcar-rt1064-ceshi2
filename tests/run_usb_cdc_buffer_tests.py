"""Exercise production CDC ring and nonblocking TX APIs with a fake USB endpoint."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "libraries/zf_driver/zf_driver_usb_cdc.c").read_bytes().decode("latin1")
start = source.index("void usb_cdc_receive_buffer_callback(void)\n{")
end = source.index("\n//--------", start)
api = source[start:end]
PREFIX = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8; typedef uint32_t uint32;
#define DATA_BUFF_SIZE 512U
#define CDC_RX_RING_SIZE 1024U
#define USB_PORTSC1_CCS_MASK 1U
#define USB_CDC_VCOM_BULK_IN_ENDPOINT 2
#define kStatus_USB_Success 0
static uint32 mask;
static uint32 __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask=1; }
static void __set_PRIMASK(uint32 m) { mask=m; }
static void __DMB(void) {}
static uint8 cdc_rx_ring[CDC_RX_RING_SIZE], cdc_tx_copy[512],usb_cdc_receive_buff[DATA_BUFF_SIZE];
static uint32 cdc_rx_write, cdc_rx_read,cdc_session,usb_cdc_receive_size;
static uint8 cdc_rx_overflow;
static struct {int attach;void*cdcAcmHandle;} s_cdcVcom;
static struct {uint32 PORTSC1;} fake_usb;
#define USB1 (&fake_usb)
static int busy,send_error,sends;
static const uint8 *dma_pointer;
static uint32 dma_length;
static uint8 usb_cdc_check_busy(void) { return (uint8)busy; }
static int USB_DeviceCdcAcmSend(void*h,int ep,uint8*p,uint32 n) {
    assert(h&&ep==2); if(send_error)return 1;
    dma_pointer=p;dma_length=n;busy=1;sends++;return 0;
}
'''
TEST = r'''
int main(void) {
    uint8 value,input[512]; memset(input,'A',sizeof(input));
    assert(!usb_cdc_ready()&&!usb_cdc_try_write(input,1));
    s_cdcVcom.attach=1;s_cdcVcom.cdcAcmHandle=&fake_usb;fake_usb.PORTSC1=1;
    assert(usb_cdc_ready());
    assert(usb_cdc_try_write(input,512)&&sends==1&&dma_length==512);
    assert(dma_pointer!=input&&dma_pointer==cdc_tx_copy);
    memset(input,'B',sizeof(input));assert(dma_pointer[0]=='A');
    assert(!usb_cdc_try_write(input,512)&&sends==1&&dma_pointer[0]=='A');
    busy=0;assert(!usb_cdc_try_write(input,513)&&!usb_cdc_try_write(input,0));
    send_error=1;assert(!usb_cdc_try_write(input,5));send_error=0;
    assert(usb_cdc_try_write(input,5)&&dma_length==5&&dma_pointer[0]=='B');busy=0;
    fake_usb.PORTSC1=0;assert(!usb_cdc_ready()&&!usb_cdc_try_write(input,1));fake_usb.PORTSC1=1;
    assert(!usb_cdc_read_byte(&value));
    for(int k=0;k<8;k++) {
        for(int i=0;i<512;i++)usb_cdc_receive_buff[i]=(uint8)i;
        usb_cdc_receive_size=512;usb_cdc_receive_buffer_callback();
        for(int i=0;i<512;i++){assert(usb_cdc_read_byte(&value)&&value==(uint8)i);}
        assert(!usb_cdc_read_byte(&value)&&!usb_cdc_rx_fault());
    }
    usb_cdc_receive_size=512;usb_cdc_receive_buffer_callback();usb_cdc_receive_buffer_callback();
    assert(usb_cdc_rx_fault()&&!usb_cdc_rx_fault()&&!usb_cdc_read_byte(&value));
    usb_cdc_receive_size=UINT32_MAX;usb_cdc_receive_buffer_callback();assert(!usb_cdc_read_byte(&value));
    mask=1;assert(!usb_cdc_read_byte(&value)&&mask==1);usb_cdc_rx_fault();assert(mask==1);
    usb_cdc_try_write(input,5);assert(mask==1);mask=0;
    assert(usb_cdc_session()==0);
    puts("PASS: production USB RX wrap/overflow/cancel, TX DMA buffer lifetime/busy/error/disconnect, interrupt state, nonblocking APIs");
}
'''
with tempfile.TemporaryDirectory(prefix="rt1064_cdc_") as directory:
    build = Path(directory)
    (build / "check.c").write_text(PREFIX + api + TEST)
    executable = build / "check.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    str(build / "check.c"), "-o", str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
