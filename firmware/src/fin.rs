// use core::marker::PhantomData;

// use teensy4_bsp::{
//     hal::{
//         self,
//         gpio::Output,
//         iomuxc::{self, consts, lpspi::Sdi},
//         lpspi::Lpspi,
//     },
//     ral,
// };

// pub struct Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk> {
//     miso: SpiMiso,
//     mosi: SpiMosi,
//     sck: SpiSck,
//     cs1: SpiCs1,
//     clk: Clk,
// }

// pub struct Fins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk, SpiN: consts::Unsigned, const GpioN: u8>
// where
//     SpiMiso: iomuxc::lpspi::Pin<Signal = Sdi, Module = SpiN>,
// {
//     spi: Lpspi<(), SpiN::USIZE>,
//     cs: Output<SpiCs1>,
//     tmr: ral::tmr::Instance<3>,
//     _phantom: PhantomData<Pins<SpiMiso, SpiMosi, SpiSck, SpiCs1, Clk>>,
// }

// impl<CsPin: iomuxc::gpio::Pin<GpioN>, ClkPin, const SpiN: u8, const GpioN: u8>
//     Fins<CsPin, ClkPin, SpiN, GpioN>
// {
//     pub fn new(
//         pins: &mut Pins,
//         lpspi: ral::lpspi::Instance<SpiN>,
//         gpio: &mut hal::gpio::Port<GpioN>,
//     ) -> Self {
//         iomuxc::lpspi::prepare(&mut pins.p11);
//         iomuxc::lpspi::prepare(&mut pins.p12);
//         iomuxc::lpspi::prepare(&mut pins.p13);
//         let spi = hal::lpspi::Lpspi::without_pins(lpspi);

//         let cs = gpio.output(CsPin);

//         let tmr = unsafe { ral::tmr::Instance::<3>::instance() };

//         Self {
//             spi,
//             cs,
//             tmr,
//             _phantom: PhantomData,
//         }
//     }
// }
