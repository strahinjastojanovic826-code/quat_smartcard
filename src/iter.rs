use crate::packed_buf::PackedQuatSlice;
use crate::quat::Quat;

/// Zero-allocation forward iterator over packed quaternary symbols.
pub struct QuatIter<'a, 'b> {
    slice: &'a PackedQuatSlice<'b>,
    pos: usize,
}

impl<'a, 'b> QuatIter<'a, 'b> {
    pub const fn new(slice: &'a PackedQuatSlice<'b>) -> Self {
        Self { slice, pos: 0 }
    }
}

impl<'a, 'b> Iterator for QuatIter<'a, 'b> {
    type Item = Quat;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.slice.len() {
            return None;
        }
        let item = self.slice.get(self.pos).ok()?;
        self.pos += 1;
        Some(item)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.slice.len().saturating_sub(self.pos);
        (remaining, Some(remaining))
    }
}

impl<'a, 'b> ExactSizeIterator for QuatIter<'a, 'b> {
    fn len(&self) -> usize {
        self.slice.len().saturating_sub(self.pos)
    }
}