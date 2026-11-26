//! Filtro One Euro (Casiez et al. 2012): quita el temblor en reposo sin añadir retraso al moverse.

#[derive(Clone, Copy)]
pub struct OneEuro {
    x: Option<f32>,
    dx: f32,
}

impl Default for OneEuro {
    fn default() -> Self {
        Self { x: None, dx: 0.0 }
    }
}

fn alpha(cutoff: f32, dt: f32) -> f32 {
    let tau = 1.0 / (2.0 * std::f32::consts::PI * cutoff);
    1.0 / (1.0 + tau / dt)
}

impl OneEuro {
    pub fn filter(&mut self, v: f32, dt: f32, min_cutoff: f32, beta: f32) -> f32 {
        let dt = dt.max(1e-3);
        let Some(prev) = self.x else {
            self.x = Some(v);
            return v;
        };
        let d = (v - prev) / dt;
        self.dx += alpha(1.0, dt) * (d - self.dx);
        let cutoff = min_cutoff + beta * self.dx.abs();
        let out = prev + alpha(cutoff, dt) * (v - prev);
        self.x = Some(out);
        out
    }
    /// Como `filter`, con el corte de la derivada elegible (más alto = velocidad más viva).
    pub fn filter_d(&mut self, v: f32, dt: f32, min_cutoff: f32, beta: f32, d_cutoff: f32) -> f32 {
        let dt = dt.max(1e-3);
        let Some(prev) = self.x else {
            self.x = Some(v);
            return v;
        };
        let d = (v - prev) / dt;
        self.dx += alpha(d_cutoff, dt) * (d - self.dx);
        let cutoff = min_cutoff + beta * self.dx.abs();
        let out = prev + alpha(cutoff, dt) * (v - prev);
        self.x = Some(out);
        out
    }
    /// Velocidad estimada (unidades/s).
    pub fn velocity(&self) -> f32 {
        self.dx
    }
    pub fn reset(&mut self) {
        self.x = None;
        self.dx = 0.0;
    }
}
