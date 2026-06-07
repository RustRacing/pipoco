use ecu_board_api::EcuOutput;
use ecu_io::{OutputTransition, OutputTransitionKind};
use ecu_sim::plant::{FixedPlantProfile, PlantProfile};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum OutputChannelKind {
    Injector,
    Ignition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ValidatedOutputChannel {
    pub(crate) kind: OutputChannelKind,
    pub(crate) channel: u8,
}

pub(crate) fn validate_x86_output_channel<const CYL: usize>(
    output: EcuOutput,
) -> Option<ValidatedOutputChannel> {
    let validated = match output {
        EcuOutput::Injector(channel) => ValidatedOutputChannel {
            kind: OutputChannelKind::Injector,
            channel: channel.get(),
        },
        EcuOutput::Ignition(channel) => ValidatedOutputChannel {
            kind: OutputChannelKind::Ignition,
            channel: channel.get(),
        },
    };

    (usize::from(validated.channel) < CYL).then_some(validated)
}

pub(crate) fn validate_scenario_output_channel(
    profile: FixedPlantProfile,
    transition: OutputTransition,
) -> Result<(), crate::DriverError> {
    let mapped = match transition.kind {
        OutputTransitionKind::Injector => profile.injector_cylinder(transition.channel),
        OutputTransitionKind::Ignition => profile.ignition_cylinder(transition.channel),
        OutputTransitionKind::Idle | OutputTransitionKind::Fan => Some(0),
    };

    mapped
        .map(|_| ())
        .ok_or(crate::DriverError::InvalidOutputChannel {
            kind: transition.kind,
            channel: transition.channel.get(),
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_board_api::EcuOutput;
    use ecu_domain::{ChannelId, Micros};
    use ecu_io::{OutputLevel, OutputTransitionKind};

    #[test]
    fn x86_output_validation_uses_target_cylinder_count() {
        assert_eq!(
            validate_x86_output_channel::<4>(EcuOutput::Injector(ChannelId::new(3))),
            Some(ValidatedOutputChannel {
                kind: OutputChannelKind::Injector,
                channel: 3,
            })
        );
        assert_eq!(
            validate_x86_output_channel::<4>(EcuOutput::Ignition(ChannelId::new(4))),
            None
        );
    }

    #[test]
    fn scenario_output_validation_uses_plant_profile_mapping() {
        let profile = FixedPlantProfile::inline_four();
        let valid = OutputTransition {
            at_us: Micros::new(100),
            kind: OutputTransitionKind::Ignition,
            channel: ChannelId::new(3),
            level: OutputLevel::High,
        };
        let invalid = OutputTransition {
            channel: ChannelId::new(4),
            ..valid
        };

        assert_eq!(validate_scenario_output_channel(profile, valid), Ok(()));
        assert_eq!(
            validate_scenario_output_channel(profile, invalid),
            Err(crate::DriverError::InvalidOutputChannel {
                kind: OutputTransitionKind::Ignition,
                channel: 4,
            })
        );
    }
}
