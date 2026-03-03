use crate::{
    common::Float,
    graph::{EdgeParams, LpfMode},
};

pub struct BMWFunction {}

fn max(a: Float, b: Float) -> Float {
    if a > b { a } else { b }
}

const OP_EXP: i32 = 2;

impl BMWFunction {
    pub fn evaluate(p: &EdgeParams, x: Float) -> Float {
        match p.mode {
            LpfMode::BPR => {
                // int_0^x toll + ff_time (1 + beta * (y+offset/capacity)^4) dy
                // = x * (toll + ff_time) + ff_time * beta / capacity^4 * int_0^y (y + offset)^4 dy
                // = x * (toll + ff_time) + ff_time * beta / capacity^4 * [ (x + offset)^5 -
                // offset^5 ] / 5

                x * (p.toll + p.ff_time)
                    + p.ff_time * p.beta / (5.0 * p.capacity.powi(4))
                        * ((x + p.offset).powi(5) - p.offset.powi(5))
            }
            LpfMode::C => (p.toll + p.ff_time) * x,
            LpfMode::OP => {
                let actual = x + p.offset;
                if actual <= p.capacity {
                    return x * (p.toll + p.ff_time);
                }

                // int_0^x toll + alpha (1 + beta * max(0, (y+offset)/gamma - 1)^delta) dy
                // = x * (toll + alpha) + alpha * beta * int_0^x max(0, (y + offset)/gamma -
                // 1)^delta dy
                // = x * (toll + alpha) + alpha * beta * int_{max(0, gamma -
                // offset)}^x ((y + offset)/gamma - 1)^delta dy
                // = x * (toll + alpha) + alpha * beta *
                //   int_{max(0, gamma - offset)}^x ((y + offset - gamma)/gamma)^delta dy
                //     with z = max(0, gamma - offset)
                // = x * (toll + alpha) + alpha * beta *
                //   [ 1/(delta+1) * ( (y + offset - gamma) / gamma)^(delta + 1) * gamma
                //   ]_{y=z}^{y=x}
                // = x * (toll + alpha) + alpha * beta / (delta+1) /
                // gamma^delta * [ (y + offset - gamma)^(delta+1) ]_{y=z}^{y=x}
                let z = max(0.0, p.capacity - p.offset);
                let overload = actual - p.capacity;
                return x * (p.toll + p.ff_time)
                    + p.ff_time * p.beta / ((OP_EXP + 1) as Float * p.capacity.powi(OP_EXP))
                        * (overload.powi(OP_EXP + 1)
                            - (z + p.offset - p.capacity).powi(OP_EXP + 1));
            }
        }
    }

    pub fn derivative(p: &EdgeParams, x: Float) -> Float {
        match p.mode {
            LpfMode::BPR => {
                p.toll + p.ff_time * (1.0 + p.beta * ((x + p.offset) / p.capacity).powi(4))
            }
            LpfMode::C => p.toll + p.ff_time,
            LpfMode::OP => {
                let actual = x + p.offset;
                if actual <= p.capacity {
                    p.toll + p.ff_time
                } else {
                    let overload = actual - p.capacity;
                    p.toll + p.ff_time * (1.0 + p.beta * (overload / p.capacity).powi(OP_EXP))
                }
            }
        }
    }
}
