use core::fmt;

use ecu_board_profiles::{
    BuildInvocationError, RecipeValidationError, TunerStudioProfileSelection,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResolveError {
    UnsupportedSelection,
    Validation(RecipeValidationError),
    BuildInvocation(BuildInvocationError),
    NoFlashCommand,
    NoTunerStudioAsset {
        selection: TunerStudioProfileSelection,
    },
}

impl fmt::Display for ResolveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSelection => write!(f, "unsupported board/recipe"),
            Self::Validation(error) => write!(f, "recipe validation failed: {error:?}"),
            Self::BuildInvocation(error) => write!(f, "build invocation failed: {error:?}"),
            Self::NoFlashCommand => write!(f, "no known flash command for board"),
            Self::NoTunerStudioAsset { selection } => {
                write!(f, "no selected TunerStudio asset for plan: {selection:?}")
            }
        }
    }
}

impl std::error::Error for ResolveError {}
