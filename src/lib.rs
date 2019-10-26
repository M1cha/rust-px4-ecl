pub use px4_ecl_sys::estimator_imuSample as ImuSample;

pub struct Ekf {
    native: Box<px4_ecl_sys::Ekf>,
}

pub struct Quatf {
    native: px4_ecl_sys::matrix_Quatf,
}

pub struct Vector3f {
    native: px4_ecl_sys::matrix_Vector3f,
}

impl From<px4_ecl_sys::matrix_Quatf> for Quatf {
    fn from(item: px4_ecl_sys::matrix_Quatf) -> Self {
        Self { native: item }
    }
}

impl<T: nalgebra::RealField + std::convert::From<f32>> Into<nalgebra::geometry::Quaternion<T>>
    for Quatf
{
    fn into(self) -> nalgebra::geometry::Quaternion<T> {
        let mut array: [f32; 4] = [0.0; 4];
        unsafe {
            px4_ecl_sys::px4_ecl_sys_helper_quatf_get(&self.native, &mut array);
        }
        nalgebra::geometry::Quaternion::new(
            array[0].into(),
            array[1].into(),
            array[2].into(),
            array[3].into(),
        )
    }
}

impl From<px4_ecl_sys::matrix_Vector3f> for Vector3f {
    fn from(item: px4_ecl_sys::matrix_Vector3f) -> Self {
        Self { native: item }
    }
}

impl From<nalgebra::Vector3<f32>> for Vector3f {
    fn from(item: nalgebra::Vector3<f32>) -> Self {
        let mut v = Self {
            native: unsafe { std::mem::zeroed() },
        };

        unsafe {
            px4_ecl_sys::px4_ecl_sys_helper_vector3f_new(&mut v.native);
        }
        unsafe {
            px4_ecl_sys::px4_ecl_sys_helper_vector3f_set(&mut v.native, item.as_ref().as_ptr());
        }

        v
    }
}

pub fn vector3f_set(vector: &mut px4_ecl_sys::matrix_Vector3f, src: nalgebra::Vector3<f32>) {
    unsafe {
        px4_ecl_sys::px4_ecl_sys_helper_vector3f_set(vector, src.as_ref().as_ptr());
    }
}

impl Into<nalgebra::Vector3<f32>> for Vector3f {
    fn into(self) -> nalgebra::Vector3<f32> {
        let mut array: [f32; 3] = [0.0; 3];
        unsafe {
            px4_ecl_sys::px4_ecl_sys_helper_vector3f_get(&self.native, &mut array);
        }
        nalgebra::Vector3::new(array[0], array[1], array[2])
    }
}

impl Drop for Vector3f {
    fn drop(&mut self) {
        unsafe {
            px4_ecl_sys::px4_ecl_sys_helper_vector3f_del(&mut self.native);
        }
    }
}

impl Ekf {
    pub fn new() -> Self {
        let mut ekf = Ekf {
            native: Box::new(px4_ecl_sys::Ekf::default()),
        };

        unsafe { px4_ecl_sys::vwrap_Ekf_constructor(&mut *ekf.native) }

        ekf
    }

    pub fn init(&mut self, timestamp: u64) -> bool {
        unsafe { px4_ecl_sys::vwrap_Ekf_init(&mut *self.native, timestamp) }
    }

    pub fn update(&mut self) -> bool {
        unsafe { px4_ecl_sys::vwrap_Ekf_update(&mut *self.native) }
    }

    pub fn set_imu_data(&mut self, imu_sample: &ImuSample) {
        unsafe { px4_ecl_sys::vwrap_Ekf_setIMUData(&mut *self.native, imu_sample) }
    }

    pub fn set_mag_data(&mut self, time_usec: u64, data: &mut [f32; 3]) {
        unsafe { px4_ecl_sys::vwrap_Ekf_setMagData(&mut *self.native, time_usec, data) }
    }

    pub fn set_baro_data(&mut self, time_usec: u64, data: f32) {
        unsafe { px4_ecl_sys::vwrap_Ekf_setBaroData(&mut *self.native, time_usec, data) }
    }

    pub fn get_quaternion(&mut self) -> Quatf {
        unsafe { (*px4_ecl_sys::vwrap_Ekf_get_quaternion(&mut *self.native)).into() }
    }

    pub fn get_velocity(&mut self, vel: &mut [f32; 3]) {
        unsafe {
            px4_ecl_sys::vwrap_Ekf_get_velocity(&mut *self.native, vel.as_mut_ptr());
        }
    }

    pub fn get_timestamp(&mut self) -> u64 {
        let mut ts: u64 = 0;
        unsafe { px4_ecl_sys::vwrap_Ekf_copy_timestamp(&mut *self.native, &mut ts) };
        ts
    }
}

impl Drop for Ekf {
    fn drop(&mut self) {
        unsafe {
            px4_ecl_sys::vwrap_Ekf_destructor(&mut *self.native);
        }
    }
}
