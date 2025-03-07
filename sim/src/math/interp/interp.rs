use std::fmt::Debug;

use itertools::{izip, Itertools};
use num_traits::Num;

#[derive(Debug, Clone, Copy)]
pub enum InterpMode<T: std::cmp::PartialOrd + Copy> {
    Extrapolate,

    FirstLast,

    Constant(T),
}

#[inline]
fn deltas<T>(p: &[T]) -> Vec<T>
where
    T: Num + Copy,
{
    p.iter().tuple_windows().map(|(&p1, &p2)| p2 - p1).collect()
}

#[inline]
fn slopes<T>(dx: &[T], dy: &[T]) -> Vec<T>
where
    T: Num + Copy,
{
    izip!(dx, dy).map(|(&dx, &dy)| dy / dx).collect()
}

#[inline]
fn intercepts<T>(x: &[T], y: &[T], m: &[T]) -> Vec<T>
where
    T: Num + Copy,
{
    izip!(x, y, m).map(|(&x, &y, &m)| y - x * m).collect()
}

#[inline]
fn prev_index<T>(x: &[T], xp: T) -> usize
where
    T: Num + PartialOrd + Copy,
{
    x.iter()
        .take_while(|&&x| x < xp)
        .enumerate()
        .last()
        .map_or(0, |(i, _)| i)
}

pub fn interp<T>(x: &[T], y: &[T], xp: T, mode: &InterpMode<T>) -> T
where
    T: Num + PartialOrd + Copy,
{
    let min_len = std::cmp::min(x.len(), y.len());

    if min_len == 0 {
        T::zero()
    } else if min_len == 1 {
        y[0]
    } else {
        let dx = deltas(&x[..min_len]);
        let dy = deltas(&y[..min_len]);

        let m = slopes(&dx, &dy);

        let c = intercepts(x, y, &m);

        let i = prev_index(x, xp).min(min_len - 2);

        let point = m[i] * xp + c[i];
        let x_limits = (&x[0], &x[min_len - 1]);
        let y_limits = (&y[0], &y[min_len - 1]);

        select_outside_point(x_limits, y_limits, &xp, point, mode)
    }
}


pub fn interp_slope<T>(x: &[T], y: &[T], xp: T, mode: &InterpMode<T>) -> T
where
    T: Num + PartialOrd + Copy + Debug,
{
    let min_len = std::cmp::min(x.len(), y.len());

    if min_len == 0 {
        T::zero()
    } else if min_len == 1 {
        T::zero()
    } else {
        let dx = deltas(&x[..min_len]);
        let dy = deltas(&y[..min_len]);

        let m = slopes(&dx, &dy);
        
        let i = prev_index(x, xp).min(min_len - 2);

        let point = m[i];
        let x_limits = (&x[0], &x[min_len - 1]);
        let y_limits = (&m[0], &m[min_len - 2]);

        select_outside_point(x_limits, y_limits, &xp, point, mode)
    }
}


fn select_outside_point<T>(
    x_limits: (&T, &T),
    y_limits: (&T, &T),
    xp: &T,
    default: T,
    mode: &InterpMode<T>,
) -> T
where
    T: Num + PartialOrd + Copy,
{
    if xp < x_limits.0 {
        match mode {
            InterpMode::Extrapolate => default,
            InterpMode::FirstLast => *y_limits.0,
            InterpMode::Constant(val) => *val,
        }
    } else if xp > x_limits.1 {
        match mode {
            InterpMode::Extrapolate => default,
            InterpMode::FirstLast => *y_limits.1,
            InterpMode::Constant(val) => *val,
        }
    } else {
        default
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interp() {
        assert_eq!(interp(&[], &[], 2.0, &InterpMode::FirstLast), 0.0);

        assert_eq!(interp(&[1.0], &[2.0], 2.0, &InterpMode::FirstLast), 2.0);

        let x = vec![0.0, 1.0, 2.0, 3.0, 4.5];
        let y = vec![0.0, 2.0, 5.0, 3.0, 2.0];

        assert_eq!(interp(&x, &y, 0.25, &InterpMode::FirstLast), 0.5);
        assert_eq!(interp(&x, &y, -1.0, &InterpMode::FirstLast), 0.0);
        assert_eq!(interp(&x, &y, 7.5, &InterpMode::FirstLast), 2.0);
    }

    #[test]
    fn test_interp_slope() {
        assert_eq!(interp_slope(&[], &[], 2.0, &InterpMode::FirstLast), 0.0);

        assert_eq!(interp_slope(&[1.0], &[2.0], 2.0, &InterpMode::FirstLast), 0.0);

        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![0.0, 2.0, 5.0, 3.0, 2.0];

        assert_eq!(interp_slope(&x, &y, 0.0, &InterpMode::Constant(0.0)), 2.0);
        assert_eq!(interp_slope(&x, &y, -1.0, &InterpMode::Constant(0.0)), 0.0);
        assert_eq!(interp_slope(&x, &y, 0.0, &InterpMode::Constant(0.0)), 2.0);
        assert_eq!(interp_slope(&x, &y, 3.5, &InterpMode::Constant(0.0)), -1.0);
    }
}
