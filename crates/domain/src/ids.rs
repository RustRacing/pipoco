/// Cylinder identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CylinderId(u8);

impl CylinderId {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u8) -> Result<Self, IdentifierError> {
        if value == 0 {
            Err(IdentifierError::Zero)
        } else {
            Ok(Self(value))
        }
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Output channel identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ChannelId(u8);

impl ChannelId {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn try_new(value: u8) -> Result<Self, IdentifierError> {
        if value == 0 {
            Err(IdentifierError::Zero)
        } else {
            Ok(Self(value))
        }
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IdentifierError {
    Zero,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_types_support_comparison() {
        assert!(CylinderId::new(1) < CylinderId::new(2));
        assert!(ChannelId::new(3) < ChannelId::new(4));
    }

    #[test]
    fn checked_id_constructors_reject_zero() {
        assert_eq!(CylinderId::try_new(0), Err(IdentifierError::Zero));
        assert_eq!(ChannelId::try_new(0), Err(IdentifierError::Zero));
        assert_eq!(CylinderId::try_new(1), Ok(CylinderId::new(1)));
        assert_eq!(ChannelId::try_new(1), Ok(ChannelId::new(1)));
    }
}
