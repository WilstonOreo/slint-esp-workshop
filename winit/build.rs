fn main() {
    // Compile the Slint file.
    // 
    // The appwindow.slint file is compiled into a Rust file that contains the UI code.
    slint_build::compile("../ui/appwindow.slint").expect("Slint build failed");
}
