fn main() {
    #[cfg(feature = "to_cxx")]
    {
        cxx_build::bridge("src/types.rs")
            .std("c++14")
            .compile("robot_behavior_cxx");

        println!("cargo:rerun-if-changed=src/ffi/to_cxx.rs");
    }
}
