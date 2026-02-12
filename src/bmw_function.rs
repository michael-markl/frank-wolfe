use crate::{common::Float, graph::EdgeParams};

pub struct BMWFunction {}

impl BMWFunction {
    pub fn evaluate(p: &EdgeParams, x: Float) -> Float {
        // int_0^x toll + ff_time (1 + beta * (y+offset/capacity)^4) dy
        // = x * (toll + ff_time) + ff_time * beta / capacity^4 * int_0^y (y + offset)^4 dy
        // = x * (toll + ff_time) + ff_time * beta / capacity^4 * [ (x + offset)^5 -
        // offset^5 ] / 5

        x * (p.toll + p.ff_time)
            + p.ff_time * p.beta / (5.0 * p.capacity.powi(4))
                * ((x + p.offset).powi(5) - p.offset.powi(5))
    }

    pub fn derivative(p: &EdgeParams, x: Float) -> Float {
        p.toll + p.ff_time * (1.0 + p.beta * ((x + p.offset) / p.capacity).powi(4))
    }
}
