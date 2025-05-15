use log::{info, warn};
use map_range::MapRange;
use quad_to_quad_transformer::{QuadTransformer, RectCorners, DEFAULT_DST_QUAD};
use serde::{Deserialize, Serialize};

use crate::{
    backend_config::{BackendConfig, CornerPoints},
    geometry_utils::distance,
    Point2D,
};

use super::clustering::Cluster2D;

/// Which part of the destination quad (ROI) to use as the origin [0,0].
/// All points sent on "smoothedTrackedPoints" will be relative to this.
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum OriginLocation {
    Corner,
    CloseCentre,
    Centre,
}

pub struct PositionRemapping {
    transformer: QuadTransformer,
    dst_quad: RectCorners,
}

impl PositionRemapping {
    pub fn new(config: &BackendConfig) -> Self {
        let dst_quad = if let Some(roi) = config.region_of_interest() {
            calculate_dst_quad(roi, config.origin_location)
        } else {
            DEFAULT_DST_QUAD
        };
        let perspective_transformer = QuadTransformer::new(
            match config.region_of_interest() {
                Some(region_of_interest) => {
                    let (c1, c2, c3, c4) = region_of_interest;
                    let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
                    Some(corners)
                }
                None => None,
            },
            if config.smoothing_use_real_units {
                info!("Using real units (mm)");
                Some(dst_quad)
            } else {
                warn!("Using normalised units");
                None
            },
            {
                if config.transform_include_outside {
                    None
                } else {
                    Some(config.transform_ignore_outside_margin)
                }
            },
        );
        PositionRemapping {
            transformer: perspective_transformer,
            dst_quad,
        }
    }

    pub fn is_ready(&self) -> bool {
        self.transformer.is_ready()
    }

    pub fn transform_clusters(&self, clusters: &[Cluster2D]) -> Vec<Cluster2D> {
        clusters
            .iter()
            .map(|c| {
                let (x, y) = self.transformer.transform(&(c.x, c.y)).unwrap();
                Cluster2D {
                    id: c.id,
                    x,
                    y,
                    size: c.size,
                }
            })
            .collect()
    }

    pub fn filter_clusters_inside(&self, clusters: &[Cluster2D]) -> Vec<Cluster2D> {
        clusters
            .iter()
            .filter(|c| self.transformer.point_is_inside_quad(&(c.x, c.y)))
            .cloned()
            .collect()
    }

    pub fn update_with_roi(
        &mut self,
        region_of_interest: &CornerPoints,
        origin_location: OriginLocation,
        use_real_units: bool,
    ) {
        let (c1, c2, c3, c4) = region_of_interest;
        let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
        self.transformer.set_new_quad(
            &corners,
            if use_real_units {
                Some(calculate_dst_quad(region_of_interest, origin_location))
            } else {
                None
            },
        );
    }

    pub fn get_dst_quad(&self) -> RectCorners {
        self.dst_quad
    }
}

/// Take a ROI, which might **not** be a rectangle, and return
/// a corresponding new "destination quad" which is a rectangle
pub fn calculate_dst_quad(roi: &CornerPoints, origin_location: OriginLocation) -> RectCorners {
    let (a, b, _c, d) = roi;
    let w = distance(a.x, a.y, b.x, b.y);
    let h = distance(a.x, a.y, d.x, d.y);

    match origin_location {
        // As seen on a normal graph (positive-y-up), corners are ordered (a,b,c,d) as if
        // "counter-clockwise" from "bottom left" (a)
        OriginLocation::Corner => [(0., 0.), (w, 0.), (w, h), (0., h)],
        OriginLocation::CloseCentre => [(-w / 2., 0.), (w / 2., 0.), (w / 2., h), (-w / 2., h)],
        OriginLocation::Centre => [
            (-w / 2., -h / 2.),
            (w / 2., -h / 2.),
            (w / 2., h / 2.),
            (-w / 2., h / 2.),
        ],
    }
}

pub fn point_remap_from_origin(
    p: Point2D,
    origin_location: OriginLocation,
    dst_quad: RectCorners,
) -> Point2D {
    let [_a, b, c, _d] = dst_quad;
    let mid_x = b.0 / 2.0;

    let (x, y) = p;
    match origin_location {
        OriginLocation::Corner => (x, y),
        OriginLocation::CloseCentre => (x.map_range(0. ..b.0, -mid_x..mid_x), y),
        OriginLocation::Centre => {
            let mid_y = c.1 / 2.0;

            (
                x.map_range(0. ..b.0, -mid_x..mid_x),
                y.map_range(0. ..c.1, -mid_y..mid_y),
            )
        }
    }
}
