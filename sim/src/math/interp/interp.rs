#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterpPos {
    Inside((usize, usize), (f64, f64)),
    Left(usize), 
    Right(usize), 
}

pub fn find_index(x: &[f64], xp: f64) -> InterpPos {
    let len = x.len();

    if xp <= x[0] {
        return InterpPos::Left(0);
    }
    if xp >= x[len - 1] {
        return InterpPos::Right(len - 1);
    }

    let idx = match x.binary_search_by(|v| v.partial_cmp(&xp).unwrap()) {
        Ok(i) => return InterpPos::Inside((i, i + 1), (0.0, x[i + 1] - x[i])),
        Err(i) => i,
    };

    let i0 = idx.saturating_sub(1);
    let i1 = idx;

    let x0 = x[i0];
    let x1 = x[i1];

    let t = (xp - x0) / (x1 - x0);
    InterpPos::Inside((i0, i1), (t, x1 - x0))
}

pub fn interpolate(y: &[f64], pos: InterpPos) -> (f64, f64) {
    match pos {
        InterpPos::Inside((i0, i1), (t, dx)) => {
            let y0 = y[i0];
            let y1 = y[i1];
            let slope = (y1 - y0) / dx;
            let interpolated_value = y0 * (1.0 - t) + y1 * t;
            (interpolated_value, slope)
        }
        InterpPos::Left(i) => {
            (y[i], 0.0)
        }
        InterpPos::Right(i) => {
            (y[i], 0.0)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_index() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.5];

        assert_eq!(find_index(&x, 0.5), InterpPos::Inside((0, 1), (0.5, 1.0)));
        assert_eq!(find_index(&x, 0.0), InterpPos::Inside((0, 1), (0.0, 1.0)));
        assert_eq!(find_index(&x, 2.0), InterpPos::Inside((2, 3), (0.0, 1.0)));
        assert_eq!(find_index(&x, -0.1), InterpPos::Left(0));
        assert_eq!(find_index(&x, 5.0), InterpPos::Right(4));
    }

    #[test]
    fn test_interpolate() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.5];
        let y = vec![0.0, 2.0, 5.0, 3.0, 2.0];

        let pos = find_index(&x, 0.5);
        assert_eq!(interpolate(&y, pos), (1.0, 2.0));

        let pos = find_index(&x, 2.0);
        assert_eq!(interpolate(&y, pos), (5.0, -2.0));

        let pos = find_index(&x, -0.1);
        assert_eq!(interpolate(&y, pos), (0.0, 0.0));

        let pos = find_index(&x, 5.0);
        assert_eq!(interpolate(&y, pos), (2.0, 0.0));
    }
}
