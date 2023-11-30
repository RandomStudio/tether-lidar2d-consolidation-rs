//! # Lidar2D Frontend
//!
//! The idea is not to launch another instance of the entire Lidar2DConsolidation backend (library), but rather
//! to share **types** between the frontend (gui) and backend ("server") applications.
//!
//! These will be two separate processes, and the frontend may be running either as a Desktop application or
//! a browser-based web application; there will always be the need for IPC (and Tether itself will facilitate this),
//! but by keeping these two in one project we can ensure that type defnitions are shared and that therefore the
//! front and back ends remain "in sync".
//!
use clap::Parser;

use env_logger::Env;
use log::debug;
use model::Model;
use tether_lidar2d_consolidation::settings::Cli;

mod model;

fn main() -> Result<(), eframe::Error> {
    let cli = Cli::parse();

    // Initialize the logger from the environment

    env_logger::Builder::from_env(Env::default().default_filter_or(&cli.log_level))
        .filter_module("paho_mqtt", log::LevelFilter::Warn)
        .filter_module("winit", log::LevelFilter::Warn)
        .filter_module("eframe", log::LevelFilter::Warn)
        .init();

    debug!("Started; args: {:?}", cli);

    let options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(1280.0, 960.0)),
        ..Default::default()
    };
    eframe::run_native(
        "Tether LIDAR2D Consolidation",
        options,
        Box::new(|_cc| Box::<Model>::default()),
    )
}
