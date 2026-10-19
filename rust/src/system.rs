//! What the settings menu shows about the box: host name and network
//! interfaces (IPv4 address, wired or Wi-Fi, the Wi-Fi network and signal
//! from NetworkManager when nmcli is there).

use std::ffi::CStr;
use std::process::Command;

pub struct Iface {
    pub name: String,
    pub wifi: bool,
    pub addr: String,
}

pub fn hostname() -> String {
    std::fs::read_to_string("/etc/hostname").map(|h| h.trim().to_string()).unwrap_or_default()
}

/// Interfaces with an IPv4 address, loopback excluded.
pub fn interfaces() -> Vec<Iface> {
    let mut list = Vec::new();
    let mut ifap: *mut libc::ifaddrs = std::ptr::null_mut();
    if unsafe { libc::getifaddrs(&mut ifap) } != 0 {
        return list;
    }
    let mut p = ifap;
    while !p.is_null() {
        let ifa = unsafe { &*p };
        p = ifa.ifa_next;
        if ifa.ifa_addr.is_null() || unsafe { (*ifa.ifa_addr).sa_family } as i32 != libc::AF_INET {
            continue;
        }
        let name = unsafe { CStr::from_ptr(ifa.ifa_name) }.to_string_lossy().into_owned();
        if name == "lo" {
            continue;
        }
        let sin = unsafe { &*(ifa.ifa_addr as *const libc::sockaddr_in) };
        let addr = std::net::Ipv4Addr::from(u32::from_be(sin.sin_addr.s_addr)).to_string();
        let wifi = std::path::Path::new(&format!("/sys/class/net/{name}/wireless")).exists();
        list.push(Iface { name, wifi, addr });
    }
    unsafe { libc::freeifaddrs(ifap) };
    list
}

/// SSID and signal (%) of the Wi-Fi network in use, without a new scan.
pub fn wifi_network() -> Option<(String, u8)> {
    let out = Command::new("nmcli")
        .args(["-t", "-f", "ACTIVE,SIGNAL,SSID", "device", "wifi", "list", "--rescan", "no"])
        .output()
        .ok()?;
    // "yes:78:My SSID"; nmcli escapes the colons inside the SSID
    String::from_utf8_lossy(&out.stdout).lines().find_map(|l| {
        let mut f = l.splitn(3, ':');
        if f.next()? != "yes" {
            return None;
        }
        let signal = f.next()?.parse().ok()?;
        Some((f.next()?.replace("\\:", ":"), signal))
    })
}
