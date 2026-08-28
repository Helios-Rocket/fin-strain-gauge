use defmt::info;
use esp_radio::{
    esp_now::{EspNow, PeerInfo, BROADCAST_ADDRESS},
    wifi::WifiController,
};

pub struct Wifi<'a> {
    esp_now: EspNow<'a>,
}

impl<'a> Wifi<'a> {
    pub fn new(controller: &'a WifiController<'a>) -> Self {
        let esp_now = controller.esp_now();
        info!("esp-now version {}", esp_now.version().unwrap());
        esp_now.set_channel(11).unwrap();

        let this = Self { esp_now };

        this
    }

    pub fn send_data(&mut self, buf: &[u8]) {
        self.esp_now
            .send(&BROADCAST_ADDRESS, buf)
            .unwrap()
            .wait()
            .unwrap();
    }

    pub fn receive_data(&mut self, buf: &mut [u8]) -> usize {
        let r = self.esp_now.receive();
        if let Some(r) = r {
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !self.esp_now.peer_exists(&r.info.src_address) {
                    self.esp_now
                        .add_peer(PeerInfo {
                            interface: esp_radio::esp_now::EspNowWifiInterface::Station,
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
            }

            let data = r.data();
            buf[0..data.len()].copy_from_slice(data);

            data.len()
        } else {
            0
        }
    }
}
