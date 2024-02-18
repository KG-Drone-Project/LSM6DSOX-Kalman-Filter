#![no_main]
#![no_std]
#![deny(unsafe_code)]
#![feature(type_alias_impl_trait)]

mod kalman;

use panic_halt as _;
use rtic::app;
//use rtic_monotonics::systick::*;

#[app(device = stm32f4xx_hal::pac, dispatchers = [SDIO] )]
mod app {

    use core::f32::consts::PI;
    use stm32f4xx_hal::{
        pac::TIM2,
        prelude::*, 
        i2c::{I2c1, Mode},
        timer::{self, Event},
    }; 

    use rtt_target::{rprintln, rtt_init_print};
    use libm::{atanf, sqrtf};

    use lsm6dsox_driver::Lsm6dsox;
    use crate::kalman::KalmanFilter;
    
    #[shared]
    struct Shared {
        x_kalman: KalmanFilter,
        y_kalman: KalmanFilter,
        timer: timer::CounterMs<TIM2>,
    }

    #[local]
    struct Local {
        imu: Lsm6dsox<I2c1>,
        i2c: I2c1,
        x_kal: f32,
        y_kal: f32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("init");
        let dp = ctx.device;

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

        let mut timer = dp.TIM2.counter_ms(&clocks);

        let id = imu.read_id(&mut i2c).unwrap();
        rprintln!("id is {:#b}: ", id);

        imu.configure_accel(&mut i2c).unwrap();
        imu.configure_gyro(&mut i2c).unwrap();

        let mut x_kalman = KalmanFilter::new();
        let mut y_kalman = KalmanFilter::new();

        let mut x_kal: f32 = 0.0;
        let mut y_kal: f32 = 0.0;

        timer.start(2000.millis()).unwrap();
        timer.listen(Event::Update);

        (Shared {x_kalman, y_kalman, timer}, Local {imu, i2c, x_kal, y_kal} )
    }

    #[idle(shared = [x_kalman, y_kalman], local = [x_kal, y_kal])]
    fn idle(mut ctx: idle::Context) -> ! {

        let mut x_kal = ctx.local.x_kal;
        let mut y_kal = ctx.local.y_kal;

        rprintln!("idle");

        loop {

            ctx.shared.x_kalman.lock(|f| {
                *x_kal = f.get_angle();
            });
            ctx.shared.y_kalman.lock(|f| {
                *y_kal = f.get_angle();
            });

            rprintln!("Kalman Filter x: {:?}, y: {:?}", *x_kal, *y_kal);
        }
    }

    #[task(shared = [x_kalman, y_kalman, timer] ,local = [imu, i2c], priority = 1)]
    async fn filter_imu_data(mut ctx: filter_imu_data::Context) {
        //rprintln!("filter");
        let delta_sec = 0.012;

        let mut accel_data:[f32; 3] = [0.0, 0.0, 0.0];
        let mut gyro_data:[f32; 3] = [0.0, 0.0, 0.0];
    
        let mut x_accel: f32 = 0.0;
        let mut y_accel: f32 = 0.0;
    
        let imu = ctx.local.imu;

        accel_data = imu.read_accel(ctx.local.i2c).unwrap();
        gyro_data = imu.read_gyro(ctx.local.i2c).unwrap();

        y_accel = atanf(accel_data[0] / sqrtf(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]) ) * 180.0 / PI;
        x_accel = atanf(accel_data[1] / sqrtf(accel_data[0] * accel_data[0] + accel_data[2] * accel_data[2]) ) * 180.0 / PI;
        
        ctx.shared.x_kalman.lock(|f| {
            f.process_posterior_state(gyro_data[0], x_accel, delta_sec);
        });

        ctx.shared.y_kalman.lock(|f| {
            f.process_posterior_state(gyro_data[1], y_accel, delta_sec);
        });

        ctx.shared.timer.lock(|f| {
            f.start(12.millis()).unwrap();
        });
    }

    #[task(binds = TIM2, shared=[timer])]
    fn timer_expired(mut ctx: timer_expired::Context) {

        ctx.shared.timer.lock(|f| {
            f.clear_all_flags();
        });

        filter_imu_data::spawn().unwrap();
    }

}