use log::{info, warn};
use map_range::MapRange;
use quad_to_quad_transformer::{QuadTransformer, RectCorners, DEFAULT_DST_QUAD};

use crate::{
    backend_config::{BackendConfig, CornerPoints},
    geometry_utils::distance,
    tracking::TrackedPoint2D,
    Point2D,
};

use super::{clustering::Cluster2D, smoothing::OriginLocation};

pub struct PositionRemapping {
    transformer: QuadTransformer,
    dst_quad: RectCorners,
}

impl PositionRemapping {
    pub fn new(config: &BackendConfig) -> Self {
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
                info!("Using real units");
                config.region_of_interest().map(calculate_dst_quad)
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
            dst_quad: if let Some(roi) = config.region_of_interest() {
                calculate_dst_quad(roi)
            } else {
                DEFAULT_DST_QUAD
            },
        }
    }

    pub fn is_ready(&self) -> bool {
        self.transformer.is_ready()
    }

    pub fn transform_clusters(&self, clusters: &[Cluster2D]) -> Vec<Point2D> {
        clusters
            .iter()
            .map(|c| self.transformer.transform(&(c.x, c.y)).unwrap())
            .collect()
    }

    pub fn filter_points_inside(&self, points: &[Point2D]) -> anyhow::Result<Vec<Point2D>> {
        self.transformer.filter_points_inside(points)
    }

    pub fn tracked_points_remap_from_origin(
        &self,
        points: &[TrackedPoint2D],
        origin_location: OriginLocation,
    ) -> Vec<TrackedPoint2D> {
        points
            .iter()
            .map(|p| {
                let TrackedPoint2D { x, y, .. } = p;
                let (remapped_x, remapped_y) =
                    point_remap_from_origin((*x, *y), origin_location, self.dst_quad);
                TrackedPoint2D {
                    x: remapped_x,
                    y: remapped_y,
                    ..*p
                }
            })
            .collect()
    }

    pub fn update_roi(&mut self, region_of_interest: &CornerPoints, use_real_units: bool) {
        let (c1, c2, c3, c4) = region_of_interest;
        let corners = [c1, c2, c3, c4].map(|c| (c.x, c.y));
        self.transformer.set_new_quad(
            &corners,
            if use_real_units {
                Some(calculate_dst_quad(region_of_interest))
            } else {
                None
            },
        );
    }

    pub fn get_dst_quad(&self) -> RectCorners {
        self.dst_quad
    }
}

pub fn calculate_dst_quad(roi: &CornerPoints) -> RectCorners {
    let (a, b, _c, d) = roi;
    let w = distance(a.x, a.y, b.x, b.y);
    let h = distance(a.x, a.y, d.x, d.y);
    [(0., 0.), (w, 0.), (w, h), (0., h)]
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
        OriginLocation::TopLeft => (x, y),
        OriginLocation::TopCentre => (x.map_range(0. ..b.0, -mid_x..mid_x), y),
        OriginLocation::BottomCentre => (
            x.map_range(0. ..b.0, -mid_x..mid_x),
            c.1 - y, // inverted
        ),
        OriginLocation::Centre => {
            let mid_y = c.1 / 2.0;

            (
                x.map_range(0. ..b.0, -mid_x..mid_x),
                y.map_range(0. ..c.1, -mid_y..mid_y),
            )
        }
    }
}
