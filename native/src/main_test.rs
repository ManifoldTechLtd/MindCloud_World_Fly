use std::{fs::File, path::PathBuf};
use clap::Parser;
use web_splats::{open_window, RenderConfig};
#[derive(Debug, Parser)]
struct Opt { input: PathBuf, #[arg(long)] no_vsync: bool }
#[pollster::main]
async fn main() {
    let opt = Opt::parse();
    let f = File::open(&opt.input).unwrap();
    open_window(f, None::<File>, RenderConfig { no_vsync: opt.no_vsync, hdr: false }, Some(opt.input), None).await;
}
