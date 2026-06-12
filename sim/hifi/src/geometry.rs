use crate::params::CylinderGeometry;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeometryModel {
    geometry: CylinderGeometry,
    crank_radius_m: f64,
    piston_area_m2: f64,
    clearance_volume_m3: f64,
}

impl GeometryModel {
    pub fn new(geometry: CylinderGeometry) -> Self {
        Self {
            crank_radius_m: geometry.stroke_m / 2.0,
            piston_area_m2: geometry.piston_area_m2(),
            clearance_volume_m3: geometry.clearance_volume_m3(),
            geometry,
        }
    }

    pub fn geometry(&self) -> CylinderGeometry {
        self.geometry
    }

    pub fn crank_radius_m(&self) -> f64 {
        self.crank_radius_m
    }

    pub fn piston_area_m2(&self) -> f64 {
        self.piston_area_m2
    }

    pub fn swept_volume_m3(&self) -> f64 {
        self.geometry.swept_volume_m3()
    }

    pub fn clearance_volume_m3(&self) -> f64 {
        self.clearance_volume_m3
    }

    pub fn piston_displacement_m(&self, theta_rad: f64) -> f64 {
        let a = self.crank_radius_m;
        let l = self.geometry.rod_length_m;
        let sin_theta = theta_rad.sin();
        let cos_theta = theta_rad.cos();
        let root = (l * l - a * a * sin_theta * sin_theta).sqrt();
        a * (1.0 - cos_theta) + l - root
    }

    pub fn volume_m3(&self, theta_rad: f64) -> f64 {
        self.clearance_volume_m3 + self.piston_area_m2 * self.piston_displacement_m(theta_rad)
    }

    pub fn dvolume_dtheta_m3_per_rad(&self, theta_rad: f64) -> f64 {
        let a = self.crank_radius_m;
        let l = self.geometry.rod_length_m;
        let sin_theta = theta_rad.sin();
        let cos_theta = theta_rad.cos();
        let root = (l * l - a * a * sin_theta * sin_theta).sqrt();
        let dx_dtheta = a * sin_theta + (a * a * sin_theta * cos_theta) / root;
        self.piston_area_m2 * dx_dtheta
    }

    pub fn wall_area_m2(&self, theta_rad: f64) -> f64 {
        let exposed_liner_height = self.piston_displacement_m(theta_rad);
        let liner_area = core::f64::consts::PI * self.geometry.bore_m * exposed_liner_height;
        2.0 * self.piston_area_m2 + liner_area
    }
}
