mod singly_linked_list {
    extern crate alloc;
    use alloc::boxed::Box;
    use core::mem;
    use core::ops::Drop;

    type Link<T> = Option<Box<Node<T>>>;

    pub struct Node<T> {
        next: Link<T>,
        val: T,
    }

    impl<T> Node<T> {
        fn new(val: T) -> Self {
            Node::<T> {
                next: None,
                val: val,
            }
        }
        fn as_ref(&self) -> &T {
            &self.val
        }
        fn as_mut(&mut self) -> &mut T {
            &mut self.val
        }
        fn take(self) -> T {
            self.val
        }
        fn insert(&mut self, val: T) -> &mut Self {
            let mut new = Box::<Node<T>>::new(Node::<T>::new(val));
            mem::swap(&mut self.next, &mut new.next);
            mem::swap(&mut self.val, &mut new.val);
            self.next = Some(new);
            self
        }
        fn remove(&mut self) -> Option<Self> {
            if self.next.is_none() {
                return None;
            }
            let mut dead = mem::replace(&mut self.next, None).unwrap();
            mem::swap(&mut self.next, &mut dead.next);
            mem::swap(&mut self.val, &mut dead.val);
            Some(*dead)
        }
    }

    pub struct List<T> {
        head: Link<T>,
        size: usize,
    }

    impl<T> Drop for List<T> {
        fn drop(&mut self) {
            while !self.is_empty() {
                self.pop();
            }
        }
    }

    impl<T> List<T> {
        pub const fn new() -> Self {
            List::<T> {
                head: None,
                size: 0,
            }
        }

        pub fn size(&self) -> usize {
            self.size
        }

        pub fn is_empty(&self) -> bool {
            assert_eq!(self.head.is_none(), self.size == 0);
            self.size() == 0
        }

        pub fn pop(&mut self) -> Option<T> {
            let fst = &mut self.head;
            if fst.is_none() {
                assert_eq!(self.size(), 0);
                return None;
            }
            let removal = fst.as_mut().unwrap().remove();
            if removal.is_none() {
                assert!(fst.as_mut().unwrap().next.is_none());
                self.size -= 1;
                assert_eq!(self.size, 0);
                return Some(mem::replace(&mut self.head, None).unwrap().take());
            }
            self.size -= 1;
            assert!(self.size > 0);
            return Some(removal.unwrap().take());
        }

        pub fn push(&mut self, val: T) -> &mut Self {
            if self.head.is_none() {
                let _ = mem::replace(&mut self.head, Some(Box::new(Node::<T>::new(val))));
                self.size += 1;
                assert!(self.head.is_some());
                return self;
            }
            (&mut self.head).as_mut().unwrap().insert(val);
            self.size += 1;
            assert!(self.head.is_some());
            self
        }
    }
}

#[cfg(test)]
mod tests {
    use super::singly_linked_list::List as SList;
    extern crate test;
    use test::{black_box, Bencher};

    #[test]
    fn test_push_pop() {
        let n = 1 << 10;
        let mut l = SList::<i32>::new();
        assert!(l.is_empty());
        for i in 0..n {
            l.push(i);
        }
        let mut count = n - 1;
        while !l.is_empty() {
            assert_eq!(l.pop().unwrap(), count);
            count -= 1;
        }
        assert_eq!(l.size(), 0);
    }

    #[bench]
    fn bench_push_pop(b: &mut Bencher) {
        let n = 1 << 16;
        let mut l = SList::<i32>::new();
        b.iter(|| {
            for i in 0..n {
                black_box(l.push(i));
            }
            while !l.is_empty() {
                black_box(l.pop());
            }
        });
    }
}
