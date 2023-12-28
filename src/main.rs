#![no_main]
#![no_std]
#![deny(unsafe_code)]
#![feature(type_alias_impl_trait)]

mod kalman;

use panic_halt as _;
use rtic::app;
use rtic_monotonics::systick::*;

#[app(device = stm32f4xx_hal::pac, dispatchers = [SDIO] )]
mod app {

    use core::f32::consts::PI;
    use rtic_monotonics::systick::Systick;
    use stm32f4xx_hal::{
        pac::TIM2,
        prelude::*, i2c::Mode, i2c::I2c1, timer::CounterMs,
    }; 

    use rtt_target::{rprintln, rtt_init_print};
    use libm::{atanf, sqrtf};

    use lsm6dsox_driver::Lsm6dsox;
    use crate::kalman::KalmanFilter;
    
    #[shared]
    struct Shared {
        timer: CounterMs<TIM2>,
    }

    #[local]
    struct Local {
        imu: Lsm6dsox<I2c1>,
        i2c: I2c1,
        x_kalman: KalmanFilter,
        y_kalman: KalmanFilter,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();

        let dp = ctx.device;
        
        let token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, token);

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.hclk(8.MHz()).freeze();
    
        let gpiob = dp.GPIOB.split();
    
        let scl = gpiob.pb6.into_open_drain_output();
        let sda = gpiob.pb7.into_open_drain_output();
    
        let mut i2c = dp.I2C1.i2c(
            (scl, sda),
            Mode::Standard { frequency: 200.kHz() },
            &clocks,
        );
        
        let imu = Lsm6dsox::new(&mut i2c).unwrap();

        let id = imu.read_id(&mut i2c).unwrap();
        rprintln!("id is {:#b}: ", id);

        imu.configure_accel(&mut i2c).unwrap();
        imu.configure_gyro(&mut i2c).unwrap();

        let mut timer = dp.TIM2.counter_ms(&clocks);

        let mut x_kalman = KalmanFilter::new();
        let mut y_kalman = KalmanFilter::new();

        filter_imu_data::spawn().ok();

        (Shared {timer}, Local {imu, i2c, x_kalman, y_kalman} )
    }

    #[task(local = [imu, i2c, x_kalman, y_kalman])]
    async fn filter_imu_data(ctx: filter_imu_data::Context) {

        let delta_sec = 0.005;

        let mut accel_data:[f32; 3] = [0.0, 0.0, 0.0];
        let mut gyro_data:[f32; 3] = [0.0, 0.0, 0.0];

        let mut x_gyro: f32 = 0.0;
        let mut y_gyro: f32 = 0.0;
        let mut z_gyro: f32 = 0.0;
    
        let mut x_accel: f32 = 0.0;
        let mut y_accel: f32 = 0.0;
    
        let mut x_kal: f32 = 0.0;
        let mut y_kal: f32 = 0.0;

        let imu = ctx.local.imu;
        let x_kalman = ctx.local.x_kalman;
        let y_kalman = ctx.local.y_kalman;

        accel_data = imu.read_accel(ctx.local.i2c).unwrap();
        gyro_data = imu.read_gyro(ctx.local.i2c).unwrap();

        x_gyro += delta_sec * gyro_data[0];
        y_gyro += delta_sec * gyro_data[1];
        z_gyro += delta_sec * gyro_data[2];

        y_accel = atanf(accel_data[0] / sqrtf(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]) ) * 180.0 / PI;
        x_accel = atanf(accel_data[1] / sqrtf(accel_data[0] * accel_data[0] + accel_data[2] * accel_data[2]) ) * 180.0 / PI;
        
        x_kal = x_kalman.get_angle(gyro_data[0], x_accel, delta_sec);
        y_kal = y_kalman.get_angle(gyro_data[1], y_accel, delta_sec);

        Systick::delay(5.millis()).await;
    }
}