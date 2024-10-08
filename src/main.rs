//! Rainbow effect color wheel using the onboard NeoPixel on an Waveshare RP2040-Zero board
//!
//! This flows smoothly through various colours on the onboard NeoPixel.
//! Uses the `ws2812_pio` driver to control the NeoPixel, which in turns uses the
//! RP2040's PIO block.
#![no_std]
#![no_main]

// use core::iter::once;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::InputPin;


extern crate alloc;
use alloc::vec::Vec;
use waveshare_rp2040_zero::rp2040_hal::fugit::ExtU32::{millis};

use embedded_alloc::LlffHeap as Heap;

use panic_halt as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use waveshare_rp2040_zero::entry;
use waveshare_rp2040_zero::{hal, Pins, XOSC_CRYSTAL_FREQ};

// use waveshare_rp2040_zero::{
//     hal::{
//         clocks::{init_clocks_and_plls, Clock},
//         pac,
//         pio::PIOExt,
//         timer::{Timer, CountDown},
//         watchdog::Watchdog,
//         Sio,
//         usb::UsbBus,
//         fugit::ExtU32,
//     },
//     Pins, XOSC_CRYSTAL_FREQ,
// };
use ws2812_pio::Ws2812;
use usbd_serial::{USB_CLASS_CDC, SerialPort};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;
use usb_device::device::{UsbVidPid, UsbDeviceBuilder};
use usb_device::class_prelude::UsbBusAllocator;


#[global_allocator]
static HEAP: Heap = Heap::empty();

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then infinitely cycles the built-in LED colour from red, to green,
/// to blue and back to red.
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Waveshare RP2040-Zero.
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
// Use the usb_bus as usual.
    //
    let mut hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 60);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        //.product("Serial port")
        // .device_class(USB_CLASS_CDC)
        .device_class(0x00)
        .composite_with_iads()
        .build();

    let mut in_pin = pins.gp15.into_pull_down_input();

    // Infinite colour wheel loop
    let on : RGB8 = (255, 255, 255).into();
    let off : RGB8 = (0, 0, 0).into();
    let mut timer = timer; // rebind to force a copy of the timer
    let mut delay = timer.count_down();

    // let mut delay_ms = |ms: u32, serial: &mut SerialPort<UsbBus>| {
    //     delay.start(ms.millis());
    //     let _ = loop {
    //         match delay.wait() {
    //             Err(nb::Error::Other(e)) => break Err(e),
    //             Err(nb::Error::WouldBlock) => {
    //                 usb_dev.poll(&mut [&mut serial])
    //             },
    //             Ok(x) => break Ok(x),
    //         };
    //     };
    // };
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }


    let mut input_count_down = timer.count_down();
    input_count_down.start(100_u32.millis());

    loop {
        // let mut scan = 0;
        if input_count_down.wait().is_ok() {
            continue;
        }

        if !usb_dev.poll(&mut [&mut serial, &mut hid]) {
            let pin_state = !in_pin.is_low().unwrap();

            if pin_state {
                let _ = serial.write(b"Hello world!\r\n");
                //     scan = 20;
                // timer.delay_ms(500);
            }
        }


        // let _ = hid.push_input(&KeyboardReport {
        //     modifier: 0,
        //     reserved: 0,
        //     leds: 0,
        //     keycodes: [scan, 0, 0, 0, 0, 0]
        // });

        // const MSG: &[u8] = "Hello, world!\r\n".as_bytes();
        // let _ = serial.write(format!("scanning {scan}").as_bytes());

        // let _ = hid.push_input(&KeyboardReport {
        //     modifier: 0,
        //     reserved: 0,
        //     leds: 0,
        //     keycodes: [scan, 0, 0, 0, 0, 0]
        // });

        // timer.delay_ms(10);
    }

    // delay_ms(500, &mut serial);

    // timer.delay_ms(500);
    // if in_pin.is_low().unwrap() {
    //     ws.write([on].iter().copied()).unwrap();
    // } else {
    //     ws.write([off].iter().copied()).unwrap();
    // }
}
