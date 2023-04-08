#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <iostream>
#include <chrono>
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#include "external/brick_source/ip_connection.h"
#include "external/brick_source/bricklet_hall_effect_v2.h"
#include "external/brick_source/brick_silent_stepper.h"

#include "NucMath/src/timeseries.h"
#include "NucMath/src/minimizer.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    IPConnection ipcon;
    HallEffectV2 he;
    SilentStepper ss;

    bool connected {false};
    QTimer *updateTimer;

private slots:
    void on_pB_connect_clicked();

    void UpdateSlot();

    void on_pB_rotate_left_clicked();

    void on_pB_rotate_for_calibration_clicked();

    void on_sB_peak1pos_valueChanged(double peak1);

    void on_pB_find_peaks_clicked();

    void on_sB_peak2pos_valueChanged(double peak2);

    void on_pB_stop_rotation_clicked();

    void on_pB_rotate_right_clicked();

    void on_sB_rot_left_degrees_valueChanged(double degrees);

    void on_sB_rot_right_degrees_valueChanged(double degrees);

    void on_pB_clear_graph_clicked();

    void on_pB_apply_calibration_values_clicked();

    void on_pB_move_to_start_position_clicked();

    void on_pB_save_graphs_clicked();

protected:

   // void paintEvent(QPaintEvent *event) override;

private:

    double rotationAngle {360};

    enum class MotorState {stopped = 0, rotLeft, rotRight, movingToZero, calibRot};

    MotorState motorState = MotorState::stopped;

    int calibrationSteps = 0;
    int startPositionSteps = 0;

    int motorMaxCurrent = 400;      // 400 mA
    int motorMaxVelocity = 500;     // steps/sec
    int motorCalibAccel = 100;      // steps/sec^2
    int motorCalibDeaccel = 250;    // steps/sec^2
    int motorCalibStepResolution = SILENT_STEPPER_STEP_RESOLUTION_64;
    int motorCalibStepsForValue = 100;

    double degreesPerStep {360.0/((360/1.8)*(40.0/12.0)*64)};
    double degreesPerStepFromCalibration {0};

    nucmath::TimeSeries timeSeries;
    nucmath::DataTable fluxStepsTable;

    std::vector<double> findPeaks(const nucmath::DataTable& fluxStepsTable);

    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
