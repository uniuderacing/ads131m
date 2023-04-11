# Rust TI ADS131M 24-bit analog-to-digital converter driver

TODO: Badges

This is an [`embedded-hal`] Rust driver for the Texas Instruments [`ADS131M`] series of simultaneously sampling
24-bit delta-sigma analog-to-digital converters.

This driver allows you to:

- Nothing Yet

## The Devices

TODO: Description

Here are the different models of ADS131M ADCs:

| Device    | Resolution | Max Sample Rate | Channels |
|-----------|------------|-----------------|----------|
| ADS131M02 | 24-bit     | 64 kSPS         | 2        |
| ADS131M03 | 24-bit     | 64 kSPS         | 3        |
| ADS131M04 | 24-bit     | 64 kSPS         | 4        |
| ADS131M06 | 24-bit     | 32 kSPS         | 6        |
| ADS131M08 | 24-bit     | 32 kSPS         | 8        |

Datasheets:

- [ADS131M02](https://www.ti.com/lit/ds/symlink/ads131m02.pdf)
- [ADS131M03](https://www.ti.com/lit/ds/symlink/ads131m03.pdf)
- [ADS131M04](https://www.ti.com/lit/ds/symlink/ads131m04.pdf)
- [ADS131M06](https://www.ti.com/lit/ds/symlink/ads131m06.pdf)
- [ADS131M08](https://www.ti.com/lit/ds/symlink/ads131m08.pdf)

## Usage Examples

TODO

```rust
unimplemented!();
```

## Support

Please feel free to submit any questions on the GitHub issues page.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Inspiration

Inspired by [ads1x1x-rs](https://github.com/eldruin/ads1x1x-rs)

## Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
[`ADS131M`]: https://www.ti.com/sitesearch/en-us/docs/universalsearch.tsp?searchTerm=ADS131M
