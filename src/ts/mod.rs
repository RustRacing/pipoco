//! TunerStudio protocol support (custom, Speeduino/rusEFI style)

pub mod outpc;
pub mod pages;
pub mod proto;
pub mod serial;
pub mod server;

pub use pages::{EcuStatePageStore, PAGE_FUEL, PAGE_IGN};
pub use server::{NoPages, OutpcProvider, PageStore, TunerstudioServer};
