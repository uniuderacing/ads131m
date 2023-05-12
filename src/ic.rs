pub trait Ic {
    const CHANNELS: usize;
}

macro_rules! ic {
    ($name:ident, $channels:literal) => {
        pub struct $name;

        impl Ic for $name {
            const CHANNELS: usize = $channels;
        }
    };
}

ic!(Ads131m02, 2);
ic!(Ads131m03, 3);
ic!(Ads131m04, 4);
ic!(Ads131m06, 6);
ic!(Ads131m08, 8);
