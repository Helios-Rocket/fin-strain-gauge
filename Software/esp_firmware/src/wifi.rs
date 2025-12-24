use core::{marker::PhantomData, ops::ControlFlow};

use defmt::println;
use esp_hal::{
    peripherals::WIFI,
    timer::timg::{TimerGroup, TimerGroupInstance},
};
use esp_radio::{
    Controller, esp_now::{BROADCAST_ADDRESS, EspNow, PeerInfo}, wifi::{Interfaces, WifiController}
};
use esp_rtos;

pub struct Wifi<'a> {
    esp_now: EspNow<'a>,
    controller: WifiController<'a>, 
}

impl<'a> Wifi<'a> {
    pub fn new(wifi: WIFI<'a>, esp_radio_ctrl: &'a Controller<'a>) -> Self {
        let (mut controller, interfaces) =
            esp_radio::wifi::new(esp_radio_ctrl, wifi, Default::default()).unwrap();
        controller.set_mode(esp_radio::wifi::WifiMode::Sta).unwrap();
        controller.start().unwrap();
        
        let esp_now = interfaces.esp_now; 
        println!("esp-now version {}", esp_now.version().unwrap());
        esp_now.set_channel(11).unwrap();

        Self { esp_now, controller}
    }

    pub fn send_data(&mut self) {
        println!("Send");
        let status = self
            .esp_now
            .send(&BROADCAST_ADDRESS, b"0123456789")
            .unwrap()
            .wait();
        println!("Send broadcast status: {:?}", status);
    }

    pub fn receive_data(&mut self) {
        let r = self.esp_now.receive();
        if let Some(r) = r {
            println!("Received {:?}", r);

            if r.info.dst_address == BROADCAST_ADDRESS {
                if !self.esp_now.peer_exists(&r.info.src_address) {
                    self.esp_now
                        .add_peer(PeerInfo {
                            interface: esp_radio::esp_now::EspNowWifiInterface::Sta,
                            peer_address: r.info.src_address,
                            lmk: (None),
                            channel: (None),
                            encrypt: (false),
                        })
                        .unwrap();
                }
                let status = self
                    .esp_now
                    .send(&r.info.src_address, b"Hello Peer")
                    .unwrap()
                    .wait();
                println!("Send hello to peer status: {:?}", status);
            }
        }
    }
}
