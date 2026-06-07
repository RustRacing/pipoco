mod handle_registry;
mod input;
mod model;
mod snapshot;
mod stepping;

#[cfg(test)]
pub(crate) use handle_registry::{
    acquire_test_lock, fabricate_wrong_nonce_handle_for_test,
    force_next_handle_generation_for_test, handle_registry_stats, max_handle_generation_for_test,
};
pub(crate) use handle_registry::{create_handle, destroy_handle, with_handle, with_state};
#[cfg(test)]
pub(crate) use model::event_less;
pub(crate) use model::{EcuSimHandle, RuntimeConfig};
