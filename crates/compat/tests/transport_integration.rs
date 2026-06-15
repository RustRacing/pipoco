//! Integration test for transport layer
//!
//! This test simulates communication between management engine and injection
//! module using the BBQueue transport.

use ecu_transport::{BbqTransport, Message, Transport};

/// Simulated management engine that sends IPW tables and config
struct ManagementEngine<T: Transport> {
    transport: T,
    table_version: u32,
}

impl<T: Transport> ManagementEngine<T> {
    fn new(transport: T) -> Self {
        Self {
            transport,
            table_version: 0,
        }
    }

    /// Send IPW table to injection module
    fn send_ipw_table(&mut self, table: [[u16; 16]; 16]) -> Result<(), String> {
        self.table_version += 1;

        let msg = Message::IpwTable {
            version: self.table_version,
            data: table,
            crc32: 0x12345678, // Simplified - should calculate real CRC
        };

        self.transport
            .send(&msg)
            .map_err(|e| format!("Failed to send table: {e:?}"))
    }

    /// Send engine configuration
    fn send_config(&mut self, num_cylinders: u8, displacement_cc: u16) -> Result<(), String> {
        let msg = Message::EngineConfig {
            num_cylinders,
            displacement_cc,
            injection_mode: 0, // Batch
            ignition_mode: 0,  // Wasted spark
            trigger_teeth: 58,
            trigger_missing: 2,
        };

        self.transport
            .send(&msg)
            .map_err(|e| format!("Failed to send config: {e:?}"))
    }

    /// Poll for incoming messages (e.g., status, errors)
    fn poll(&mut self) {
        self.transport.poll();

        // Check for incoming messages
        while let Some(msg) = self.transport.try_receive() {
            match msg {
                Message::TriggerTiming {
                    gap_period_us,
                    synced,
                    ..
                } => {
                    if synced {
                        // Calculate exact RPM from gap period
                        let exact_rpm = 2_068_966_u32 / gap_period_us;
                        println!("Injection module synced, RPM: {exact_rpm}");
                    }
                }
                Message::Error {
                    node_id,
                    error_code,
                    severity,
                    ..
                } => {
                    eprintln!("Error from node {node_id}: code {error_code} severity {severity}");
                }
                Message::Heartbeat {
                    node_id,
                    uptime_seconds,
                    ..
                } => {
                    println!("Heartbeat from node {node_id} at {uptime_seconds}s");
                }
                _ => {}
            }
        }
    }
}

/// Simulated injection module that receives tables and sends timing data
struct InjectionModule<T: Transport> {
    transport: T,
    node_id: u8,
    current_table_version: u32,
    current_table: [[u16; 16]; 16],
    uptime_seconds: u32,
}

impl<T: Transport> InjectionModule<T> {
    fn new(transport: T, node_id: u8) -> Self {
        Self {
            transport,
            node_id,
            current_table_version: 0,
            current_table: [[1000; 16]; 16],
            uptime_seconds: 0,
        }
    }

    /// Send trigger timing data to management engine
    fn send_timing(
        &mut self,
        gap_period_us: u32,
        tooth_position: u8,
        synced: bool,
    ) -> Result<(), String> {
        let msg = Message::TriggerTiming {
            gap_period_us,
            tooth_period_us: (gap_period_us / 2) as u16,
            tooth_position,
            synced,
            timestamp_us: self.uptime_seconds * 1_000_000,
        };

        self.transport
            .send(&msg)
            .map_err(|e| format!("Failed to send timing: {e:?}"))
    }

    /// Send heartbeat
    fn send_heartbeat(&mut self) -> Result<(), String> {
        let msg = Message::Heartbeat {
            node_id: self.node_id,
            uptime_seconds: self.uptime_seconds,
            status: 0,
            error_count: 0,
            cpu_usage: 50,
        };

        self.transport
            .send(&msg)
            .map_err(|e| format!("Failed to send heartbeat: {e:?}"))
    }

    /// Poll for incoming messages (tables, config, commands)
    fn poll(&mut self) {
        self.transport.poll();

        while let Some(msg) = self.transport.try_receive() {
            match msg {
                Message::IpwTable {
                    version,
                    data,
                    crc32,
                } => {
                    println!("Received IPW table version {version} (CRC: 0x{crc32:08x})");
                    self.current_table_version = version;
                    self.current_table = data;
                }
                Message::EngineConfig {
                    num_cylinders,
                    displacement_cc,
                    ..
                } => {
                    println!(
                        "Received engine config: {num_cylinders} cylinders, {displacement_cc}cc"
                    );
                }
                Message::CmdReset { target_node_id }
                    if target_node_id == self.node_id || target_node_id == 0xFF =>
                {
                    println!("Reset command received");
                    // Reset logic here
                }
                Message::CmdReset { .. } => {}
                _ => {}
            }
        }
    }

    fn get_table_version(&self) -> u32 {
        self.current_table_version
    }
}

#[test]
fn test_management_to_injection_communication() {
    // Create transport pair (can only be called once per test binary)
    let (mgmt_transport, inj_transport) = BbqTransport::create_pair().unwrap();

    // Create modules
    let mut management = ManagementEngine::new(mgmt_transport);
    let mut injection = InjectionModule::new(inj_transport, 2);

    // Management sends engine config
    management.send_config(4, 2000).unwrap();

    // Injection receives config
    injection.poll();

    // Management sends IPW table
    let mut test_table = [[1000u16; 16]; 16];
    test_table[0][0] = 1500; // Unique value for testing
    management.send_ipw_table(test_table).unwrap();

    // Injection receives table
    injection.poll();
    assert_eq!(injection.get_table_version(), 1);

    // Injection sends timing data
    injection.send_timing(2000, 1, true).unwrap();

    // Management receives timing
    management.poll();

    // Check stats
    let mgmt_stats = management.transport.stats();
    let inj_stats = injection.transport.stats();

    assert_eq!(mgmt_stats.tx_count, 2); // Config + table
    assert_eq!(inj_stats.tx_count, 1); // Timing
    assert_eq!(mgmt_stats.rx_count, 1); // Timing
    assert_eq!(inj_stats.rx_count, 2); // Config + table

    // === Test 2: Bidirectional communication ===
    println!("\nTest 2: Bidirectional communication");

    // Send multiple messages in both directions
    for i in 0..5 {
        // Management → Injection
        management.send_config(4, 2000).unwrap();
        injection.poll();

        // Injection → Management
        injection.send_timing(2000 - (i * 10), 1, true).unwrap();
        management.poll();

        // Injection → Management (heartbeat)
        injection.send_heartbeat().unwrap();
        management.poll();
    }

    // Verify more messages were received
    let mgmt_stats2 = management.transport.stats();
    let inj_stats2 = injection.transport.stats();

    assert!(mgmt_stats2.tx_count > mgmt_stats.tx_count);
    assert!(inj_stats2.tx_count > inj_stats.tx_count);

    // === Test 3: Large table transmission ===
    println!("\nTest 3: Large table transmission");

    // Create table with unique values
    let mut test_table2 = [[0u16; 16]; 16];
    for (i, row) in test_table2.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = 2000 + (i * 16 + j) as u16;
        }
    }

    // Send second table
    management.send_ipw_table(test_table2).unwrap();
    injection.poll();

    assert_eq!(injection.get_table_version(), 2);
    assert_eq!(injection.current_table[0][0], 2000);
    assert_eq!(injection.current_table[5][10], 2000 + 90);
    assert_eq!(injection.current_table[15][15], 2000 + 255);

    println!("\nAll integration tests passed!");
    let mgmt_stats = management.transport.stats();
    let inj_stats = injection.transport.stats();
    println!(
        "Management TX: {tx}, RX: {rx}",
        tx = mgmt_stats.tx_count,
        rx = mgmt_stats.rx_count
    );
    println!(
        "Injection TX: {tx}, RX: {rx}",
        tx = inj_stats.tx_count,
        rx = inj_stats.rx_count
    );
}
