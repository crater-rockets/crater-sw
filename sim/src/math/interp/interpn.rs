use std::{array, cell::RefCell, iter::Sum};

use num_traits::{Float, float::TotalOrder};

struct Lattice<const D: usize> {
    size: [usize; D],

    strides: [usize; D],
    hypercube_offsets: Vec<usize>,
}

impl<const D: usize> Lattice<D> {
    pub fn new(size: [usize; D]) -> Self {
        let strides = Self::calc_strides(&size);

        Self {
            size,
            hypercube_offsets: Self::calc_hypercube_offsets(&strides),
            strides,
        }
    }
}

impl<const D: usize> Lattice<D> {
    fn calc_strides(size: &[usize; D]) -> [usize; D] {
        let mut strides = [0; D];
        strides[D - 1] = 1usize;

        for d in (0..D - 1).rev() {
            strides[d] = strides[d + 1] * size[d + 1];
        }

        strides
    }

    fn calc_hypercube_offsets(strides: &[usize; D]) -> Vec<usize> {
        let n: usize = 1 << D;
        let mut offsets = vec![0; n];

        for i in 0..n {
            for d in 0..D {
                offsets[i] += strides[d] * ((i >> d) & 0x1);
            }
        }

        offsets
    }

    #[allow(unused)]
    pub fn size(&self) -> &[usize; D] {
        &self.size
    }

    #[allow(unused)]
    pub fn strides(&self) -> &[usize; D] {
        &self.strides
    }

    pub fn hypercube_offsets(&self) -> &[usize] {
        &self.hypercube_offsets
    }

    pub fn flat_index(&self, index: &[usize; D]) -> usize {
        debug_assert!(self.size.iter().enumerate().all(|(i, s)| index[i] < *s));

        index
            .iter()
            .zip(self.strides.iter())
            .map(|(i, s)| i * s)
            .sum()
    }
}

pub struct Interpolator<T, const D: usize> {
    axes: [Vec<T>; D],
    axes_steps: [Vec<T>; D],
    lattice: Lattice<D>,

    mut_alloc: RefCell<InterpolatorAlloc<T>>,
}

struct InterpolatorAlloc<T> {
    offsets: Vec<usize>,
    weigths: Vec<T>,
}

impl<T: Float> InterpolatorAlloc<T> {
    fn new(nelem: usize) -> Self {
        InterpolatorAlloc {
            offsets: vec![0; nelem],
            weigths: vec![T::zero(); nelem],
        }
    }
}

impl<T: Float + TotalOrder + Sum, const D: usize> Interpolator<T, D> {
    pub fn new(axes: [&[T]; D]) -> Option<Self> {
        // Check that data size matches

        let size: [usize; D] = array::from_fn(|i| axes[i].len());

        let axes: [Vec<T>; D] = array::from_fn(|i| axes[i].to_vec());
        let axes_steps =
            array::from_fn(|i| axes[i].windows(2).map(|pair| pair[1] - pair[0]).collect());

        Some(Self {
            axes,
            axes_steps,
            lattice: Lattice::new(size),
            mut_alloc: RefCell::new(InterpolatorAlloc::new(1 << D)),
        })
    }

    fn find_edge_index(&self, state: &[T; D]) -> [usize; D] {
        // TODO: Memory
        let indices: [usize; D] = array::from_fn(|i| {
            self.axes[i][1..self.axes[i].len() - 1]
                .binary_search_by(|v| v.total_cmp(&state[i]))
                .unwrap_or_else(|e| e)
        });

        indices
    }

    fn calc_normalized_interp_point(&self, state: &[T; D], indices: &[usize; D]) -> [T; D] {
        let x: [T; D] = array::from_fn(|i| {
            let is = indices[i];
            let v = (state[i] - self.axes[i][is]) / self.axes_steps[i][is];
            v.min(T::one()).max(T::zero())
        });

        x
    }

    /// Algorithm as described by Gupta et al. https://www.jmlr.org/papers/volume17/15-243/15-243.pdf
    fn interpn_weights<'a>(x: &[T; D], weights_out: &'a mut [T]) -> &'a [T] {
        weights_out[0] = T::one();

        let mut wi = 1usize;
        for d in 0..D {
            let sd = 1 << d;

            for k in 0..sd {
                weights_out[wi] = x[d] * weights_out[k];

                weights_out[k] = (T::one() - x[d]) * weights_out[k];
                wi += 1;
            }
        }

        weights_out
    }

    pub fn interpn<'a, const N: usize>(
        &self,
        state: &[T; D],
        data: &[&[T]; N],
        interp_out: &'a mut [T; N],
    ) {
        let indices = self.find_edge_index(state);
        let x = self.calc_normalized_interp_point(state, &indices);

        let mut alloc = self.mut_alloc.borrow_mut();

        Self::interpn_weights(&x, &mut alloc.weigths);

        let first_vertex = self.lattice.flat_index(&indices);
        alloc
            .offsets
            .iter_mut()
            .zip(self.lattice.hypercube_offsets())
            .for_each(|(to, o)| *to = first_vertex + o);

        for (i, data) in data.iter().enumerate() {
            interp_out[i] = alloc
                .offsets
                .iter()
                .zip(alloc.weigths.iter())
                .map(|(o, w)| data[*o] * *w)
                .sum();
        }
    }
}
