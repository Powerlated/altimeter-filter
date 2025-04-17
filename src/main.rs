use eframe::egui::{self};
use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints};
use std::env;

fn main() -> eframe::Result {
    let args: Vec<String> = env::args().collect();
    let open_data_log_path = args.get(1);

    let native_options = eframe::NativeOptions {
        vsync: false,
        viewport: egui::ViewportBuilder::default()
            .with_maximized(true),
        ..Default::default()
    };
    eframe::run_native(
        "",
        native_options,
        Box::new(|cc| Ok(Box::new(App::new(cc, open_data_log_path.cloned())))),
    )
}

struct App {
    points: Vec<PlotPoint>,
}

impl App {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>, open_data_log_path: Option<String>) -> Self {
        App {
            points: Default::default()
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            Plot::new("My Plot")
                .legend(Legend::default())
                .default_x_bounds(0., 100.)
                .default_y_bounds(0., 10000.)
                .show(ui, |plot_ui| {
                    plot_ui
                        .line(Line::new("curve", PlotPoints::Borrowed(&self.points)).name("curve"));
                });
        });
    }
}
