use ecu_board_profiles::{BoardSelection, FirmwareBuildInvocation, FirmwareBuildPlan};

use crate::{
    errors::ResolveError, resolve_metadata_and_recipe, FIRMWARE_REGISTRY, SUPPORTED_INVOCATIONS,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareAlias {
    pub alias: &'static str,
    pub board: &'static str,
    pub recipe: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareSelection {
    pub board: &'static str,
    pub recipe: &'static str,
}

pub fn resolve_alias(alias: &str) -> Result<FirmwareSelection, ResolveError> {
    crate::SUPPORTED_ALIASES
        .iter()
        .find(|entry| entry.alias == alias)
        .map(|entry| FirmwareSelection {
            board: entry.board,
            recipe: entry.recipe,
        })
        .ok_or(ResolveError::UnsupportedSelection)
}

pub fn resolve_invocation(
    board: &str,
    recipe: &str,
) -> Result<FirmwareBuildInvocation, ResolveError> {
    let (_, invocation) = resolve_invocation_with_board(board, recipe)?;
    Ok(invocation)
}

pub(crate) fn resolve_invocation_with_board(
    board: &str,
    recipe: &str,
) -> Result<(&'static str, FirmwareBuildInvocation), ResolveError> {
    let (board_id, plan) = resolve_plan_with_board(board, recipe)?;
    let invocation = plan
        .cargo_build_invocation()
        .map_err(ResolveError::BuildInvocation)?;
    Ok((board_id, invocation))
}

pub(crate) fn resolve_plan(board: &str, recipe: &str) -> Result<FirmwareBuildPlan, ResolveError> {
    let (_, plan) = resolve_plan_with_board(board, recipe)?;
    Ok(plan)
}

pub(crate) fn resolve_flash_chip(board: &str, recipe: &str) -> Result<&'static str, ResolveError> {
    let canonical = canonical_selection(board, recipe)?;
    let entry = FIRMWARE_REGISTRY
        .iter()
        .find(|entry| entry.board == canonical.board && entry.recipe == canonical.recipe)
        .ok_or(ResolveError::UnsupportedSelection)?;
    entry.flash_chip.ok_or(ResolveError::NoFlashCommand)
}

pub(crate) fn resolve_plan_with_board(
    board: &str,
    recipe: &str,
) -> Result<(&'static str, FirmwareBuildPlan), ResolveError> {
    let canonical = canonical_selection(board, recipe)?;
    let (board_id, metadata, recipe) =
        resolve_metadata_and_recipe(canonical.board, canonical.recipe)?;
    let recipe = recipe.for_board(BoardSelection::Named(board_id));
    let plan = recipe
        .resolve_build_plan(metadata)
        .map_err(ResolveError::Validation)?;
    Ok((board_id, plan))
}

pub(crate) fn canonical_selection(
    board: &str,
    recipe: &str,
) -> Result<FirmwareSelection, ResolveError> {
    if let Some(alias) = crate::SUPPORTED_ALIASES
        .iter()
        .find(|entry| entry.alias == board)
    {
        if alias.recipe == recipe {
            return Ok(FirmwareSelection {
                board: alias.board,
                recipe: alias.recipe,
            });
        }
    }

    if SUPPORTED_INVOCATIONS
        .iter()
        .any(|(supported_board, supported_recipe)| {
            *supported_board == board && *supported_recipe == recipe
        })
    {
        let Some((board, recipe)) = canonical_supported_invocation(board, recipe) else {
            return Err(ResolveError::UnsupportedSelection);
        };
        return Ok(FirmwareSelection { board, recipe });
    }

    Err(ResolveError::UnsupportedSelection)
}

fn canonical_supported_invocation(
    board: &str,
    recipe: &str,
) -> Option<(&'static str, &'static str)> {
    FIRMWARE_REGISTRY
        .iter()
        .find(|entry| entry.listed && entry.board == board && entry.recipe == recipe)
        .map(|entry| (entry.board, entry.recipe))
}
