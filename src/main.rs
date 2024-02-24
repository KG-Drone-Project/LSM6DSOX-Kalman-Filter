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
    use heapless::spsc::{Consumer, Producer, Queue};

    use stm32f4xx_hal::{
        i2c::{I2c1, Mode},
        pac::TIM2,
        prelude::*,
        timer::{self, Event},
    };

    use libm::{atanf, sqrtf};
    use rtt_target::{rprintln, rtt_init_print};

    use crate::kalman::KalmanFilter;
    use lsm6dsox_driver::Lsm6dsox;

    #[shared]
    struct Shared {
        timer: timer::CounterMs<TIM2>,
    }

    #[local]
    struct Local {
        imu: Lsm6dsox<I2c1>,
        i2c: I2c1,
        x_kalman: KalmanFilter,
        y_kalman: KalmanFilter,

        p: Producer<'static, [f32; 2], 5>,
        c: Consumer<'static, [f32; 2], 5>
    }

    #[init(local = [q: Queue<[f32; 2], 5> = Queue::new()])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("init");

        let (p, c) = ctx.local.q.split();

        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.hclk(8.MHz()).freeze();

        let gpiob = dp.GPIOB.split();

        let scl = gpiob.pb6.into_open_drain_output();
        let sda = gpiob.pb7.into_open_drain_output();

        let mut i2c = dp.I2C1.i2c(
            (scl, sda),
            Mode::Standard {
                frequency: 200.kHz(),
            },
            &clocks,
        );

        let imu = Lsm6dsox::new(&mut i2c).unwrap();

        let mut timer = dp.TIM2.counter_ms(&clocks);

        let id = imu.read_id(&mut i2c).unwrap();
        rprintln!("id is {:#b}: ", id);

        imu.configure_accel(&mut i2c).unwrap();
        imu.configure_gyro(&mut i2c).unwrap();

        let x_kalman = KalmanFilter::new();
        let y_kalman = KalmanFilter::new();

        timer.start(2000.millis()).unwrap();
        timer.listen(Event::Update);

        (
            Shared {
                timer,
            },
            Local {
                imu,
                i2c,
                x_kalman,
                y_kalman,

                p,
                c,
            },
        )
    }

    #[idle(local = [c])]
    fn idle(mut ctx: idle::Context) -> ! {


        rprintln!("idle");

        loop {

            //rprintln!("Kalman Filter x: {:?}, y: {:?}", *x_kal, *y_kal);
            if let Some(data) = ctx.local.c.dequeue() {
                rprintln!("Data: {:?}", data);
            }

        }
    }

    #[task(shared = [timer] ,local = [imu, i2c, x_kalman, y_kalman, p], priority = 1)]
    async fn filter_imu_data(mut ctx: filter_imu_data::Context) {
        //rprintln!("filter");

        ctx.shared.timer.lock(|f| {
            f.start(12.millis()).unwrap();
        });
        let delta_sec = 0.012;

        let mut accel_data: [f32; 3] = [0.0, 0.0, 0.0];
        let mut gyro_data: [f32; 3] = [0.0, 0.0, 0.0];

        let mut x_accel: f32 = 0.0;
        let mut y_accel: f32 = 0.0;

        let imu = ctx.local.imu;

        accel_data = imu.read_accel(ctx.local.i2c).unwrap();
        gyro_data = imu.read_gyro(ctx.local.i2c).unwrap();

        y_accel = atanf(accel_data[0] / sqrtf(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2])) * 180.0 / PI;
        x_accel = atanf(accel_data[1] / sqrtf(accel_data[0] * accel_data[0] + accel_data[2] * accel_data[2])) * 180.0/ PI;

        ctx.local.x_kalman.process_posterior_state(gyro_data[0], x_accel, delta_sec);
        ctx.local.y_kalman.process_posterior_state(gyro_data[1], y_accel, delta_sec);

        match ctx.local.p.enqueue([ctx.local.x_kalman.get_angle(), ctx.local.y_kalman.get_angle()]) {
            Ok(()) => {
                //rprintln!("Data sent");
            }

            Err(err) => {
                // Other errors occurred, handle them appropriately
                // Example: println!("Error occurred while enqueueing data: {:?}", err);
                rprintln!("IMU Data failed to send: {:?}", err);
            }
        }
        
    }

    #[task(binds = TIM2, shared=[timer])]
    fn timer_expired(mut ctx: timer_expired::Context) {
        ctx.shared.timer.lock(|f| {
            f.clear_all_flags();
        });

        filter_imu_data::spawn();
    }
}
