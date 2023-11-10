use log::info;
use tether_agent::{TetherAgent, TetherAgentOptionsBuilder};
use tether_lidar2d_consolidation::settings::Cli;

use clap::Parser;

pub struct Model {
    pub tether_agent: TetherAgent,
}

impl Default for Model {
    fn default() -> Self {
        let cli = Cli::parse();

        let tether_agent = TetherAgentOptionsBuilder::new("lidar2dFrontend")
            .build()
            .expect("failed to init+connect Tether Agent");

        info!("Lidar2D Frontend started OK");

        Model { tether_agent }
    }
}

impl eframe::App for Model {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Wow it works");
        });
    }
}
