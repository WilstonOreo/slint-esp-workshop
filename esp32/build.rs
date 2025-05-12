fn main() {
    embuild::espidf::sysenv::output();
    slint_build::compile_with_config(
        "../ui/appwindow.slint",
        slint_build::CompilerConfiguration::new()
            .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer)
            .with_sdf_fonts(true),
    )
    .unwrap();
}
