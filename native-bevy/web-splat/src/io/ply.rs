use anyhow::Ok;
use half::f16;
use ply_rs::ply;

use std::io::{self, BufReader, Read, Seek};

use byteorder::{BigEndian, ByteOrder, LittleEndian, ReadBytesExt};
use cgmath::{InnerSpace, Point3, Quaternion, Vector3};

use crate::{
    pointcloud::Gaussian,
    utils::{build_cov, sh_deg_from_num_coefs, sigmoid},
};

use super::{GenericGaussianPointCloud, PointCloudReader};

pub struct PlyReader<R: Read + Seek> {
    header: ply_rs::ply::Header,
    reader: BufReader<R>,
    sh_deg: u32,
    num_points: usize,
    mip_splatting: Option<bool>,
    kernel_size: Option<f32>,
    background_color: Option<[f32; 3]>,
    has_normals: bool,
    scale_before_sh: bool,
}

impl<R: io::Read + io::Seek> PlyReader<R> {
    pub fn new(reader: R) -> Result<Self, anyhow::Error> {
        let mut reader = BufReader::new(reader);
        let parser = ply_rs::parser::Parser::<ply_rs::ply::DefaultElement>::new();
        let header = parser.read_header(&mut reader).unwrap();
        let sh_deg = Self::file_sh_deg(&header)?;
        let num_points = Self::num_points(&header)?;
        let mip_splatting = Self::mip_splatting(&header)?;
        let kernel_size = Self::kernel_size(&header)?;
        let background_color = Self::background_color(&header)
            .map_err(|e| log::warn!("could not parse background_color: {}", e))
            .unwrap_or_default();
        let has_normals = Self::has_normals(&header);
        let scale_before_sh = Self::scale_before_sh(&header);
        Ok(Self {
            header,
            reader,
            sh_deg,
            num_points,
            mip_splatting,
            kernel_size,
            background_color,
            has_normals,
            scale_before_sh,
        })
    }

    /// Check if the vertex element has normal properties (nx, ny, nz).
    fn has_normals(header: &ply::Header) -> bool {
        header.elements.get("vertex")
            .map(|v| v.properties.contains_key("nx"))
            .unwrap_or(false)
    }

    /// Detect property ordering: returns true if scale comes before f_dc in the header.
    fn scale_before_sh(header: &ply::Header) -> bool {
        if let Some(vertex) = header.elements.get("vertex") {
            let keys: Vec<&String> = vertex.properties.keys().collect();
            let scale_pos = keys.iter().position(|k| k.as_str() == "scale_0");
            let fdc_pos = keys.iter().position(|k| k.as_str() == "f_dc_0");
            match (scale_pos, fdc_pos) {
                (Some(s), Some(f)) => s < f,
                _ => false,
            }
        } else {
            false
        }
    }

    fn read_line<B: ByteOrder>(
        &mut self,
        sh_deg: usize,
    ) -> anyhow::Result<(Gaussian, [[f16; 3]; 16])> {
        let mut pos = [0.; 3];
        self.reader.read_f32_into::<B>(&mut pos)?;

        // Skip normals if present
        if self.has_normals {
            let mut _normals = [0.; 3];
            self.reader.read_f32_into::<B>(&mut _normals)?;
        }

        let num_coefs = (sh_deg + 1) * (sh_deg + 1);
        let mut sh: [[f32; 3]; 16] = [[0.; 3]; 16];
        let mut opacity_raw = 0f32;
        let mut scale = Vector3::new(1.0f32, 1.0, 1.0);
        let mut rot = Quaternion::new(1.0f32, 0.0, 0.0, 0.0);

        if self.scale_before_sh {
            // Format: x,y,z, scale_0,scale_1,scale_2, f_dc_0,f_dc_1,f_dc_2, opacity, rot_0..3, f_rest_*
            let s0 = self.reader.read_f32::<B>()?.exp();
            let s1 = self.reader.read_f32::<B>()?.exp();
            let s2 = self.reader.read_f32::<B>()?.exp();
            scale = Vector3::new(s0, s1, s2);

            self.reader.read_f32_into::<B>(&mut sh[0])?; // f_dc_0,1,2

            opacity_raw = self.reader.read_f32::<B>()?;

            let r0 = self.reader.read_f32::<B>()?;
            let r1 = self.reader.read_f32::<B>()?;
            let r2 = self.reader.read_f32::<B>()?;
            let r3 = self.reader.read_f32::<B>()?;
            rot = Quaternion::new(r0, r1, r2, r3).normalize();

            // f_rest (SH higher order)
            if num_coefs > 1 {
                let mut sh_rest = [0.; 15 * 3];
                self.reader.read_f32_into::<B>(&mut sh_rest[..(num_coefs - 1) * 3])?;
                for i in 0..(num_coefs - 1) {
                    for j in 0..3 {
                        sh[i + 1][j] = sh_rest[j * (num_coefs - 1) + i];
                    }
                }
            }
        } else {
            // Original format: x,y,z, [nx,ny,nz], f_dc_0,1,2, f_rest_*, opacity, scale_0,1,2, rot_0,1,2,3
            self.reader.read_f32_into::<B>(&mut sh[0])?;
            let mut sh_rest = [0.; 15 * 3];
            if num_coefs > 1 {
                self.reader.read_f32_into::<B>(&mut sh_rest[..(num_coefs - 1) * 3])?;
            }
            for i in 0..(num_coefs - 1) {
                for j in 0..3 {
                    sh[i + 1][j] = sh_rest[j * (num_coefs - 1) + i];
                }
            }

            opacity_raw = self.reader.read_f32::<B>()?;

            let s0 = self.reader.read_f32::<B>()?.exp();
            let s1 = self.reader.read_f32::<B>()?.exp();
            let s2 = self.reader.read_f32::<B>()?.exp();
            scale = Vector3::new(s0, s1, s2);

            let r0 = self.reader.read_f32::<B>()?;
            let r1 = self.reader.read_f32::<B>()?;
            let r2 = self.reader.read_f32::<B>()?;
            let r3 = self.reader.read_f32::<B>()?;
            rot = Quaternion::new(r0, r1, r2, r3).normalize();
        }

        let opacity = sigmoid(opacity_raw);
        let cov = build_cov(rot, scale);

        return Ok((
            Gaussian::new(
                Point3::from(pos).cast().unwrap(),
                f16::from_f32(opacity),
                cov.map(|x| f16::from_f32(x)),
            ),
            sh.map(|x| x.map(|y| f16::from_f32(y))),
        ));
    }

    fn file_sh_deg(header: &ply::Header) -> Result<u32, anyhow::Error> {
        let num_sh_coefs = header.elements["vertex"]
            .properties
            .keys()
            .filter(|k| k.starts_with("f_"))
            .count();

        let file_sh_deg = sh_deg_from_num_coefs(num_sh_coefs as u32 / 3).ok_or(anyhow::anyhow!(
            "number of sh coefficients {num_sh_coefs} cannot be mapped to sh degree"
        ))?;
        Ok(file_sh_deg)
    }

    fn num_points(header: &ply::Header) -> Result<usize, anyhow::Error> {
        Ok(header
            .elements
            .get("vertex")
            .ok_or(anyhow::anyhow!("missing element vertex"))?
            .count as usize)
    }

    fn mip_splatting(header: &ply::Header) -> Result<Option<bool>, anyhow::Error> {
        Ok(header
            .comments
            .iter()
            .find(|c| c.contains("mip"))
            .map(|c| c.split('=').last().unwrap().to_lowercase().parse::<bool>())
            .transpose()?)
    }
    fn kernel_size(header: &ply::Header) -> Result<Option<f32>, anyhow::Error> {
        Ok(header
            .comments
            .iter()
            .find(|c| c.contains("kernel_size"))
            .map(|c| c.split('=').last().unwrap().parse::<f32>())
            .transpose()?)
    }

    fn background_color(header: &ply::Header) -> anyhow::Result<Option<[f32; 3]>> {
        header
            .comments
            .iter()
            .find(|c| c.contains("background_color"))
            .map(|c| {
                let value = c.split('=').last();
                let parts = value.map(|c| {
                    c.split(",")
                        .map(|v| v.parse::<f32>())
                        .collect::<Result<Vec<f32>, _>>()
                });
                parts.map_or_else(
                    || Err(anyhow::anyhow!("could not parse:")),
                    |x| {
                        x.map_err(|e| anyhow::anyhow!("could not parse: {}", e))
                            .map(|x| [x[0], x[1], x[2]])
                    },
                )
            })
            .transpose()
    }
}

impl<R: io::Read + io::Seek> PointCloudReader for PlyReader<R> {
    fn read(&mut self) -> Result<GenericGaussianPointCloud, anyhow::Error> {
        let mut gaussians = Vec::with_capacity(self.num_points);
        let mut sh_coefs = Vec::with_capacity(self.num_points);
        match self.header.encoding {
            ply_rs::ply::Encoding::Ascii => todo!("acsii ply format not supported"),
            ply_rs::ply::Encoding::BinaryBigEndian => {
                for _ in 0..self.num_points {
                    let (g, s) = self.read_line::<BigEndian>(self.sh_deg as usize)?;
                    gaussians.push(g);
                    sh_coefs.push(s);
                }
            }
            ply_rs::ply::Encoding::BinaryLittleEndian => {
                for _ in 0..self.num_points {
                    let (g, s) = self.read_line::<LittleEndian>(self.sh_deg as usize)?;
                    gaussians.push(g);
                    sh_coefs.push(s);
                }
            }
        };
        return Ok(GenericGaussianPointCloud::new(
            gaussians,
            sh_coefs,
            self.sh_deg,
            self.num_points,
            self.kernel_size,
            self.mip_splatting,
            self.background_color,
            None,
            None,
        ));
    }

    fn magic_bytes() -> &'static [u8] {
        "ply".as_bytes()
    }

    fn file_ending() -> &'static str {
        "ply"
    }
}
