use altimeter_filter::pressure_mbar_to_ft;
use csv::DeserializeRecordsIter;
use eframe::egui::{self};
use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};
use serde::Deserialize;
use std::env;
use std::fs::File;

fn main() -> eframe::Result {
    let args: Vec<String> = env::args().collect();
    let open_data_log_path = args.get(1).unwrap().clone();

    let native_options = eframe::NativeOptions {
        vsync: false,
        viewport: egui::ViewportBuilder::default().with_maximized(true),
        ..Default::default()
    };
    eframe::run_native(
        "",
        native_options,
        Box::new(|cc| Ok(Box::new(App::new(cc, open_data_log_path)))),
    )
}

#[derive(Deserialize)]
pub struct LogPacketV1Csv {
    pub time_boot_ms: u32,
    pub ms5607_pressure_mbar: f32,
    pub ms5607_temperature_c: f32,
    pub bmi323_accel_x_mps2_fc_frame: f32,
    pub bmi323_accel_y_mps2_fc_frame: f32,
    pub bmi323_accel_z_mps2_fc_frame: f32,
    pub bmi323_gyro_x_rps_fc_frame: f32,
    pub bmi323_gyro_y_rps_fc_frame: f32,
    pub bmi323_gyro_z_rps_fc_frame: f32,
    pub adxl375_accel_x_mps2_fc_frame: f32,
    pub adxl375_accel_y_mps2_fc_frame: f32,
    pub adxl375_accel_z_mps2_fc_frame: f32,
}

struct App {
    points: Vec<PlotPoint>,
}

impl App {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>, open_data_log_path: String) -> Self {
        let mut points = Vec::new();

        let mut reader = csv::Reader::from_path(open_data_log_path).unwrap();
        let iter: DeserializeRecordsIter<File, LogPacketV1Csv> = reader.deserialize();

        let mut t0 = None;
        for i in iter {
            if let Ok(result) = i {
                let p: LogPacketV1Csv = result;
                let t = p.time_boot_ms as f32 / 1000.;
                if t0.is_none() {
                    t0 = Some(t);
                }
                
                let ft;
                unsafe {
                    ft = pressure_mbar_to_ft(p.ms5607_pressure_mbar);
                }
                println!("{t}");
                points.push(PlotPoint {
                    x: (t - t0.unwrap()) as f64,
                    y: ft as f64,
                })
            }
        }

        println!("Points: {}", points.len());

        App { points }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            Plot::new("My Plot")
                .legend(Legend::default())
                .show(ui, |plot_ui| {
                    plot_ui
                        .line(Line::new("Altitude", PlotPoints::Borrowed(&self.points)).name("Altitude"));
                });
        });
    }
}
