

pub enum Message {
    SetSpeed(f32),
    SetPosition(i32),
    SetPID(u8, f32, f32, f32),
    Enable(bool),
    Home,
}


impl Message {
    pub fn from_bytes(bytes: &[u8]) -> Option<Message> {
        if bytes.len() < 1 {
            return None;
        }

        match bytes[0] {
            b's' => {
                let speed = if let Ok(num) = bytes[1..5].try_into() {
                    f32::from_be_bytes(num)
                } else {
                    return None
                };

                if bytes[5] == b's' {
                    Some(Message::SetSpeed(speed))
                } else {
                    None
                }
            }
            b'p' => {
                let pos = if let Ok(num) = bytes[1..5].try_into() {
                    i32::from_be_bytes(num)
                } else {
                    return None
                };

                if bytes[5] == b'p' {
                    Some(Message::SetPosition(pos))
                } else {
                    None
                }
            }
            b'e' => {
                let en = bytes[1] != 0;
                if bytes[2] == b'e' {
                    Some(Message::Enable(en))
                } else {
                    None
                }
            }
            b'h' => {
                if bytes[1] == 1 && bytes[2] == b'h' {
                    Some(Message::Home)
                } else {
                    None
                }
            }

            _ => { None }
        }
    }
}