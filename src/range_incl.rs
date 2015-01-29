use std::num::Int;

pub struct RangeIncl<T: Int> {
    state: Option<T>,
    end: T,
}

pub fn range_incl<T: Int>(begin: T, end: T) -> RangeIncl<T> {
    RangeIncl { state: Some(begin), end: end }
}

trait One : Int {
    fn my_one(_: Option<Self>) -> Self {
        Int::one()
    }
}
impl<T> One for T where T: Int {}

impl<T: Int> Iterator for RangeIncl<T> {
    type Item = T;
    fn next(&mut self) -> Option<<Self as Iterator>::Item> {
        match self.state {
            Some(current) => {
                self.state =
                    if current == self.end { None }
                    else { Some(current + One::my_one(None::<T>)) };
                Some(current)
            },
            None => {
                None
            }
        }
    }
}
