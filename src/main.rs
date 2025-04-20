use altimeter_filter::*;
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

#[derive(Default)]
struct App {
    vec_packets: Vec<LogPacketV1Csv>,

    vec_alt: Vec<PlotPoint>,
    vec_alt_filtered: Vec<PlotPoint>,
    vec_vel_filtered: Vec<PlotPoint>,
    vec_acc_filtered: Vec<PlotPoint>,
    vec_jerk_filtered: Vec<PlotPoint>,
    vec_time_to_apogee_s_filtered: Vec<PlotPoint>,

    link_axis: Vec2b,
    link_cursor: Vec2b,

    process_variance_pre_apogee: [f32; 4],
    process_variance_post_apogee: [f32; 4],
    observation_variance: [f32; 2],
}

impl App {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>, open_data_log_path: String) -> Self {
        let mut vec_packets = Vec::new();

        let mut reader = csv::Reader::from_path(open_data_log_path).unwrap();
        let iter: DeserializeRecordsIter<File, LogPacketV1Csv> = reader.deserialize();

        for i in iter {
            if let Ok(result) = i {
                let p: LogPacketV1Csv = result;
                vec_packets.push(p);
                // println!("{ft_filtered}");
            }
        }

        App {
            vec_packets,

            link_axis: Vec2b::new(true, false),
            link_cursor: Vec2b::new(true, false),

            process_variance_pre_apogee: [1., 1., 1., 1.],
            process_variance_post_apogee: [1., 1., 1., 1.],
            observation_variance: [1., 1.],

            ..Default::default()
        }
    }

    pub fn init_filter(&mut self) {
        unsafe {
            let ft = pressure_mbar_to_ft(self.vec_packets[0].ms5607_pressure_mbar);
            let m = ft * 0.3048;
            AltimeterFilterInit(m, 0.);
        }
    }

    pub fn run_filter(&mut self) {
        self.vec_alt.clear();
        self.vec_alt_filtered.clear();
        self.vec_vel_filtered.clear();
        self.vec_acc_filtered.clear();
        self.vec_jerk_filtered.clear();
        self.vec_time_to_apogee_s_filtered.clear();

        let mut t0 = None;

        for p in &self.vec_packets {
            let t = p.time_boot_ms as f32 / 1000.;

            let ft;
            let m;
            unsafe {
                ft = pressure_mbar_to_ft(p.ms5607_pressure_mbar);
                m = ft * 0.3048;
            }

            if t0.is_none() {
                t0 = Some(t);
            }

            let filter_out: AltimeterFilterOutput;
            unsafe {
                filter_out = AltimeterFilterProcess(m);
            }
            let ft_filtered = filter_out.altitude_m / 0.3048;

            self.vec_alt.push(PlotPoint {
                x: (t - t0.unwrap()) as f64,
                y: ft as f64,
            });

            self.vec_alt_filtered.push(PlotPoint {
                x: (t - t0.unwrap()) as f64,
                y: ft_filtered as f64,
            });

            self.vec_vel_filtered.push(PlotPoint {
                x: (t - t0.unwrap()) as f64,
                y: filter_out.velocity_mps as f64,
            });

            self.vec_acc_filtered.push(PlotPoint {
                x: (t - t0.unwrap()) as f64,
                y: filter_out.acceleration_mps2 as f64,
            });

            self.vec_jerk_filtered.push(PlotPoint {
                x: (t - t0.unwrap()) as f64,
                y: filter_out.jerk_mps3 as f64,
            });

            self.vec_time_to_apogee_s_filtered.push(PlotPoint {
                x: (t - t0.unwrap()) as f64,
                y: filter_out.time_to_apogee_s as f64,
            });
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("Sidebar").show(ctx, |ui| {
            let process_labels = ["Altitude", "Velocity", "Acceleration", "Jerk"];
            let observation_labels = ["Pressure Altitude", "Assumed Acceleration"];

            self.init_filter();

            ui.label("Process Variances (pre-apogee)");
            egui::Grid::new("st")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for i in 0..self.process_variance_pre_apogee.len() {
                        ui.label(process_labels[i]);
                        ui.add(
                            egui::DragValue::new(&mut self.process_variance_pre_apogee[i])
                                .speed(0.01)
                                .range(0.001..=10000.),
                        );
                        ui.end_row();

                        unsafe {
                            AltimeterFilterSetProcessVariancePreApogee(
                                i as i32,
                                self.process_variance_pre_apogee[i],
                            );
                        }
                    }
                });

            ui.label("Process Variances (post-apogee)");
            egui::Grid::new("st")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for i in 0..self.process_variance_post_apogee.len() {
                        ui.label(process_labels[i]);
                        ui.add(
                            egui::DragValue::new(&mut self.process_variance_post_apogee[i])
                                .speed(0.01)
                                .range(0.001..=10000.),
                        );
                        ui.end_row();

                        unsafe {
                            AltimeterFilterSetProcessVariancePostApogee(
                                i as i32,
                                self.process_variance_post_apogee[i],
                            );
                        }
                    }
                });

            ui.label("Observation Variances");
            egui::Grid::new("obs")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for i in 0..self.observation_variance.len() {
                        ui.label(observation_labels[i]);
                        ui.add(
                            egui::DragValue::new(&mut self.observation_variance[i])
                                .speed(0.01)
                                .range(0.001..=10000.),
                        );
                        ui.end_row();

                        unsafe {
                            AltimeterFilterSetObservationVariance(
                                i as i32,
                                self.observation_variance[i],
                            );
                        }
                    }
                });

            self.run_filter();
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let link_group_id = ui.id().with("linked_demo");

            Plot::new("My Plot")
                .legend(Legend::default())
                .height(450.0)
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
                .height(450.0)
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

            Plot::new("My Plot3")
                .legend(Legend::default())
                .height(500.0)
                .link_axis(link_group_id, self.link_axis)
                .link_cursor(link_group_id, self.link_cursor)
                .show(ui, |plot_ui| {
                    plot_ui.line(
                        Line::new(
                            "Time to Apogee (s) (filtered)",
                            PlotPoints::Borrowed(&self.vec_time_to_apogee_s_filtered),
                        )
                        .name("Time to Apogee (s) (filtered)"),
                    );
                });
        });
    }
}
