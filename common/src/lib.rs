
pub mod weather;

pub mod sensor;

/// Convenience helper for passing the last of a value between threads. For example from a thread
/// interfacing with a sensor to another one processing the data.
#[derive(Clone, Default)]
struct ValueStore<T>(std::sync::Arc<std::sync::Mutex<Option<T>>>);

impl<T: Clone> ValueStore<T> {
    /// Sets `value` as the last value.
    ///
    /// # Panics
    ///
    /// If the locking the interally used mutex fails.
    fn set(&self, value: T) {
        let mut data = self.0.lock().unwrap();
        let _ = data.insert(value);
    }

    /// Gets the stored value.
    ///
    /// # Panics
    ///
    /// If the locking of the mutex fails
    fn get(&self) -> Option<T> {
        let mut data = self.0.lock().unwrap();
        data.take()
    }
}