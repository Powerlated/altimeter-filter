use altimeter_filter::{
    AltimeterFilterGetAcceleration, AltimeterFilterGetVelocity, AltimeterFilterGetJerk, AltimeterFilterInit,
    AltimeterFilterProcess, pressure_mbar_to_ft,
};
use csv::DeserializeRecordsIter;
use eframe::egui::{self, Vec2b};
use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};
use serde::Deserialize;
use std::env;
use std::fs::File;

fn main() -> eframe::Result {
    let args: Vec<String> = env::args().collect();
    let open_data_log_path = args.get(1).unwrap().clone();
    // let open_data_log_path =
        // "../irec/flight-data-logs/test-flight-4-12-2025_flight-only.csv".to_string();

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
    vec_alt: Vec<PlotPoint>,
    vec_alt_filtered: Vec<PlotPoint>,
    vec_vel_filtered: Vec<PlotPoint>,
    vec_acc_filtered: Vec<PlotPoint>,
    vec_jerk_filtered: Vec<PlotPoint>,

    link_axis: Vec2b,
    link_cursor: Vec2b,
}

impl App {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>, open_data_log_path: String) -> Self {
        let mut vec_alt = Vec::new();
        let mut vec_alt_filtered = Vec::new();
        let mut vec_vel_filtered = Vec::new();
        let mut vec_acc_filtered = Vec::new();
        let mut vec_jerk_filtered = Vec::new();


        let mut reader = csv::Reader::from_path(open_data_log_path).unwrap();
        let iter: DeserializeRecordsIter<File, LogPacketV1Csv> = reader.deserialize();

        let mut t0 = None;
        for i in iter {
            if let Ok(result) = i {
                let p: LogPacketV1Csv = result;
                let t = p.time_boot_ms as f32 / 1000.;

                let ft;
                let m;
                unsafe {
                    ft = pressure_mbar_to_ft(p.ms5607_pressure_mbar);
                    m = ft * 0.3048;
                }

                if t0.is_none() {
                    t0 = Some(t);

                    unsafe {
                        AltimeterFilterInit(m, 0.);
                        // AltimeterFilterProcess(m) / 0.3048;
                    }
                }

                let ft_filtered;
                let vel_filtered;
                let acc_filtered;
                let jerk_filtered;
                unsafe {
                    ft_filtered = AltimeterFilterProcess(m) / 0.3048;
                    vel_filtered = AltimeterFilterGetVelocity();
                    acc_filtered = AltimeterFilterGetAcceleration();
                    jerk_filtered = AltimeterFilterGetJerk();
                }

                vec_alt.push(PlotPoint {
                    x: (t - t0.unwrap()) as f64,
                    y: ft as f64,
                });

                vec_alt_filtered.push(PlotPoint {
                    x: (t - t0.unwrap()) as f64,
                    y: ft_filtered as f64,
                });

                vec_vel_filtered.push(PlotPoint {
                    x: (t - t0.unwrap()) as f64,
                    y: vel_filtered as f64,
                });

                vec_acc_filtered.push(PlotPoint {
                    x: (t - t0.unwrap()) as f64,
                    y: acc_filtered as f64,
                });

                vec_jerk_filtered.push(PlotPoint {
                    x: (t - t0.unwrap()) as f64,
                    y: jerk_filtered as f64,
                });


                // println!("{ft_filtered}");
            }
        }

        println!("Points: {}", vec_alt.len());

        App {
            vec_alt,
            vec_alt_filtered,
            vec_vel_filtered,
            vec_acc_filtered,
            vec_jerk_filtered,

            link_axis: Vec2b::new(true, false),
            link_cursor: Vec2b::new(true, false)
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let link_group_id = ui.id().with("linked_demo");

            Plot::new("My Plot")
                .legend(Legend::default())
                .height(500.0)
                .link_axis(link_group_id, self.link_axis)
                .link_cursor(link_group_id, self.link_cursor)
                .show(ui, |plot_ui| {
                    plot_ui.line(
                        Line::new("Altitude (ft)", PlotPoints::Borrowed(&self.vec_alt))
                            .name("Altitude (ft)"),
                    );
                    plot_ui.line(
                        Line::new(
                            "Altitude (ft) (filtered)",
                            PlotPoints::Borrowed(&self.vec_alt_filtered),
                        )
                        .name("Altitude (ft) (filtered)"),
                    );
                });

            Plot::new("My Plot2")
                .legend(Legend::default())
                .height(500.0)
                .link_axis(link_group_id, self.link_axis)
                .link_cursor(link_group_id, self.link_cursor)
                .show(ui, |plot_ui| {
                    plot_ui.line(
                        Line::new(
                            "Velocity (m/s) (filtered)",
                            PlotPoints::Borrowed(&self.vec_vel_filtered),
                        )
                        .name("Velocity (m/s) (filtered)"),
                    );
                    plot_ui.line(
                        Line::new(
                            "Acceleration (m/s²) (filtered)",
                            PlotPoints::Borrowed(&self.vec_acc_filtered),
                        )
                        .name("Acceleration (m/s²) (filtered)"),
                    );
                    plot_ui.line(
                        Line::new(
                            "Jerk (m/s³) (filtered)",
                            PlotPoints::Borrowed(&self.vec_jerk_filtered),
                        )
                        .name("Jerk (m/s³) (filtered)"),
                    );
                });
        });
    }
}
