// Main loop
#include "rapidcsv.h"
#include "AltimeterFilter.h"
#include <SDL3/SDL.h>
#include <implot.h>
#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <imgui_impl_sdl3.h>
#include <ranges>
#include <tuple>

struct DataEntry
{
    float time_boot_ms;
    float accel_z_mps2;
    float ms5607_pressure_mbar;
};

std::vector<DataEntry> entries;
std::vector<float> v_filtered_t;
std::vector<float> v_original_alt_ft;
std::vector<float> v_filtered_alt_ft;
std::vector<float> v_filtered_vel;
std::vector<float> v_predicted_apogee_ft;
std::vector<float> v_earth_rel_accel;
std::vector<float> v_filtered_accel;
std::vector<float> v_predicted_accel;
std::vector<float> v_apogee_ft_if_deployed_now;
float t_apogee;
float alt_apogee;
float t_burnout;
float alt_burnout;
float t_deploy;
float alt_deploy;

float process_variance_pre_apogee[] = {45.397, 0., 3.365, 0.540};
float process_variance_post_apogee[] = {1., 1., 1., 1.};
float observation_variance[] = {89.524, 3.810};

bool graphOutOfDate = true;

void SetupVisualizer()
{
    ImPlot::CreateContext();

    rapidcsv::Document doc("test-flight-4-12-2025_flight-only.csv");

    for (const auto &[time_boot_ms,
                      accel_z_mps2,
                      ms5607_pressure_mbar] :
         std::views::zip(doc.GetColumn<float>("time_boot_ms"),
                         doc.GetColumn<float>("adxl375_accel_z_mps2_fc_frame"),
                         doc.GetColumn<float>("ms5607_pressure_mbar")))
    {
        entries.push_back({
            .time_boot_ms = time_boot_ms,
            .accel_z_mps2 = accel_z_mps2,
            .ms5607_pressure_mbar = ms5607_pressure_mbar,
        });

        auto ft = pressure_mbar_to_ft(ms5607_pressure_mbar);
        v_original_alt_ft.push_back(ft);
    }
}

typedef boost::array<double, 4> state_type;

// this is for asteria II IREC 2025
float rocket_mass_kg = 17.63996;
float drag_model_scale = 4.0;
float target_apogee_ft = 6000;
#define MARGIN_FT 100

float drag_newtons_airbrakes_100(float mach_number)
{
    float x = mach_number;
    return 4.21 - 36.6*x + 130*pow(x,2);
}

float drag_newtons_airbrakes_stowed(float mach_number)
{
    float x = mach_number;
    return 1.39 - 12.7*x + 56.4*pow(x,2);
}


void projectile_motion_rocket_airbrakes_stowed(const state_type &x, state_type &dxdt, double t)
{
    auto speed_of_sound = 343; // TODO: Speed of sound varies with altitude
    float speed = sqrtf(powf(x[2], 2) + powf(x[3], 2));
    auto mach_number = speed / speed_of_sound;
    float drag_newtons = drag_newtons_airbrakes_stowed(mach_number) * drag_model_scale;
    float drag_y_newtons = -drag_newtons * 1;
    dxdt[0] = x[2]; // = v_x
    dxdt[1] = x[3]; // = v_y
    dxdt[2] = 0 / rocket_mass_kg;
    dxdt[3] = -G + drag_y_newtons / rocket_mass_kg;
}

void projectile_motion_rocket_airbrakes_100(const state_type &x, state_type &dxdt, double t)
{
    auto speed_of_sound = 343; // TODO: Speed of sound varies with altitude
    float speed = sqrtf(powf(x[2], 2) + powf(x[3], 2));
    auto mach_number = speed / speed_of_sound;
    float drag_newtons = drag_newtons_airbrakes_100(mach_number) * drag_model_scale;
    float drag_y_newtons = -drag_newtons * 1;
    dxdt[0] = x[2]; // = v_x
    dxdt[1] = x[3]; // = v_y
    dxdt[2] = 0 / rocket_mass_kg;
    dxdt[3] = -G + drag_y_newtons / rocket_mass_kg;
}

float apogee_ft_if_deployed_now(float altitude_m, float velocity_mps)
{
    boost::numeric::odeint::runge_kutta_cash_karp54<state_type> rk;
    state_type x{0, altitude_m, 0, velocity_mps}; // initial condition
    const double dt = 0.1;
    double t_apogee_predictor = 0;
    while (x[3] > 0.0)
    {
        rk.do_step(projectile_motion_rocket_airbrakes_100, x, t_apogee_predictor, dt);
        t_apogee_predictor += dt;
    }

    return x[1] / 0.3048;
}

void RunFilter()
{
    v_filtered_t.reserve(entries.size());
    v_filtered_accel.reserve(entries.size());
    v_filtered_alt_ft.reserve(entries.size());
    v_filtered_vel.reserve(entries.size());
    v_earth_rel_accel.reserve(entries.size());
    v_predicted_apogee_ft.reserve(entries.size());
    v_predicted_accel.reserve(entries.size());
    v_apogee_ft_if_deployed_now.reserve(entries.size());
    v_filtered_t.clear();
    v_filtered_alt_ft.clear();
    v_filtered_vel.clear();
    v_earth_rel_accel.clear();
    v_predicted_apogee_ft.clear();
    v_filtered_accel.clear();
    v_predicted_accel.clear();
    v_apogee_ft_if_deployed_now.clear();

    t_apogee = 0.0;
    alt_apogee = 0.0;
    t_burnout = 0.0;
    alt_burnout = 0.0;
    t_deploy = 0.0;
    alt_deploy = 0.0;

    auto ft = pressure_mbar_to_ft(entries.at(0).ms5607_pressure_mbar);
    auto m = ft * 0.3048;
    AltimeterFilterInit(m, 0.);

    for (int i = 0; i < STATE_LEN; i++)
    {
        AltimeterFilterSetProcessVariancePreApogee(i, process_variance_pre_apogee[i]);
        AltimeterFilterSetProcessVariancePostApogee(i, process_variance_post_apogee[i]);
    }

    for (int i = 0; i < OBSERVATION_LEN; i++)
    {
        AltimeterFilterSetObservationVariance(i, observation_variance[i]);
    }

    float avg_raw_accel_z = 0.0;
    for (int i = 0; i < 100; i++)
    {
        auto raw_accel_z = entries.at(i).accel_z_mps2;
        assert(std::abs(raw_accel_z) < 20);
        avg_raw_accel_z += raw_accel_z;
    }
    avg_raw_accel_z /= 100.0;
    float accel_z_bias = avg_raw_accel_z - 9.81;
    // printf("Accel z bias: %f\n", accel_z_bias);

    t_apogee = 0.0;

    auto t0_ms = entries.at(0).time_boot_ms;
    for (const auto &e : entries)
    {
        auto ft = pressure_mbar_to_ft(e.ms5607_pressure_mbar);
        auto m = ft * 0.3048;

        auto earth_rel_accel = (e.accel_z_mps2 - accel_z_bias) - G;
        auto filter_out = AltimeterFilterProcess(m, earth_rel_accel);
        auto ft_filtered = filter_out.altitude_m / 0.3048;

        auto t = (e.time_boot_ms - t0_ms) / 1000.0;
        v_filtered_t.push_back(t);
        v_filtered_alt_ft.push_back(ft_filtered);
        v_filtered_vel.push_back(filter_out.velocity_mps);
        v_earth_rel_accel.push_back(earth_rel_accel);
        v_filtered_accel.push_back(filter_out.acceleration_mps2);
        v_predicted_accel.push_back(-drag_newtons_airbrakes_stowed(filter_out.velocity_mps / 343) / rocket_mass_kg - G);

        if (t_apogee == 0.0 && AltimeterFilterGetFlightStage() == STAGE_APOGEE)
        {
            t_apogee = t;
            alt_apogee = ft_filtered;
        }

        if (t_burnout == 0.0 && AltimeterFilterGetFlightStage() == STAGE_BURNOUT)
        {
            t_burnout = t;
            alt_burnout = ft_filtered;
        }

        // Apogee prediction
        if (AltimeterFilterGetFlightStage() == STAGE_BURNOUT)
        {
            boost::numeric::odeint::runge_kutta_cash_karp54<state_type> rk;
            state_type x{0, filter_out.altitude_m, 0, filter_out.velocity_mps}; // initial condition
            const double dt = 0.1;
            double t_apogee_predictor = t;
            while (x[3] > 0.0)
            {
                rk.do_step(projectile_motion_rocket_airbrakes_stowed, x, t_apogee_predictor, dt);
                t_apogee_predictor += dt;
            }

            auto if_now = apogee_ft_if_deployed_now(filter_out.altitude_m, filter_out.velocity_mps);
            if (if_now <= target_apogee_ft + MARGIN_FT && if_now >= target_apogee_ft - MARGIN_FT)  {
                if (t_deploy == 0) {
                    t_deploy = t;
                    alt_deploy = filter_out.altitude_m / 0.3048;
                }
            }
            v_predicted_apogee_ft.push_back(x[1] / 0.3048);
            v_apogee_ft_if_deployed_now.push_back(if_now);
        }
        else
        {
            v_predicted_apogee_ft.push_back(0);
            v_apogee_ft_if_deployed_now.push_back(0);
        }
    }

    assert(v_filtered_t.size() == v_filtered_alt_ft.size());
    assert(v_original_alt_ft.size() == v_filtered_alt_ft.size());
    assert(v_earth_rel_accel.size() == v_filtered_alt_ft.size());
    assert(v_filtered_vel.size() == v_filtered_alt_ft.size());
    assert(v_predicted_apogee_ft.size() == v_filtered_alt_ft.size());
    assert(v_filtered_accel.size() == v_filtered_alt_ft.size());
    // assert(v_predicted_accel.size() == v_filtered_alt_ft.size());
}

void ShowVisualizer()
{
    if (graphOutOfDate)
    {
        graphOutOfDate = false;
        RunFilter();
    }

    ImVec2 size = ImGui::GetContentRegionAvail();
    size.y /= 3;
    size.y -= ImGuiStyleVar_ItemSpacing;

    if (ImGui::Begin("Plots"))
    {
        static ImPlotRect lims(0, 1, 0, 1);

        if (ImPlot::BeginAlignedPlots("AlignedGroup"))
        {
            if (ImPlot::BeginPlot("Altitude", size))
            {
                ImPlot::SetupAxes("Time (s)", "Altitude (ft)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Altitude, original", v_filtered_t.data(), v_original_alt_ft.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Altitude, filtered", v_filtered_t.data(), v_filtered_alt_ft.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::SetNextLineStyle(ImVec4(.99, .99, .2, 1));
                ImPlot::PlotLine("Predicted apogee", v_filtered_t.data(), v_predicted_apogee_ft.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Apogee if airbrake deployed now", v_filtered_t.data(), v_apogee_ft_if_deployed_now.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                float apogeeLineX[2] = {lims.X.Min, lims.X.Max};
                float apogeeLineY[2] = {alt_apogee, alt_apogee};
                ImPlot::SetNextLineStyle(ImVec4(.2, .8, .2, 1));
                ImPlot::PlotLine("Final apogee", apogeeLineX, apogeeLineY, 2, ImPlotLineFlags_None);
                ImPlot::Annotation(t_apogee, alt_apogee, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Apogee");
                ImPlot::Annotation(t_burnout, alt_burnout, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Burnout");
                ImPlot::Annotation(t_deploy, alt_deploy, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Airbrake Deploy");
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Velocity", size))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity (m/s)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Velocity (m/s)", v_filtered_t.data(), v_filtered_vel.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::Annotation(t_apogee, alt_apogee, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Apogee");
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Acceleration", size))
            {
                ImPlot::SetupAxes("Time (s)", "Acceleration (m/s^2)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Sensed Acceleration (m/s^2)", v_filtered_t.data(), v_earth_rel_accel.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Filtered Acceleration (m/s^2)", v_filtered_t.data(), v_filtered_accel.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Predicted Acceleration (m/s^2)", v_filtered_t.data(), v_predicted_accel.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::EndPlot();
            }
            ImPlot::EndAlignedPlots();
        }

        ImGui::End();
    }

    if (ImGui::Begin("Options"))
    {
        ImGui::Text("Process Variances");
        graphOutOfDate |= ImGui::SliderFloat("Altitude Pre-apogee", &process_variance_pre_apogee[0], 0.0f, 100.0f);
        graphOutOfDate |= ImGui::SliderFloat("Velocity Pre-apogee", &process_variance_pre_apogee[1], 0.0f, 10.0f);
        graphOutOfDate |= ImGui::SliderFloat("Acceleration Pre-apogee", &process_variance_pre_apogee[2], 0.0f, 10.0f);
        graphOutOfDate |= ImGui::SliderFloat("Jerk Pre-apogee", &process_variance_pre_apogee[3], 0.0f, 10.0f);

        ImGui::Separator();
        ImGui::Text("Observation Variances");
        graphOutOfDate |= ImGui::SliderFloat("Barometer", &observation_variance[0], 0.0f, 100);
        graphOutOfDate |= ImGui::SliderFloat("Accelerometer", &observation_variance[1], 0.0f, 100);

        ImGui::Separator();
        ImGui::Text("Other");
        graphOutOfDate |= ImGui::SliderFloat("Rocket Mass (kg)", &rocket_mass_kg, 0.0f, 25);
        graphOutOfDate |= ImGui::SliderFloat("Drag Model Scale", &drag_model_scale, 0.5f, 20.0f);
        graphOutOfDate |= ImGui::SliderFloat("Target Apogee (ft)", &target_apogee_ft, 6000, 10000);

        ImGui::End();
    }
}