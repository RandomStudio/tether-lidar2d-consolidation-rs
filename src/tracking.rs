pub mod tracking {
    use na::{Matrix3, Point2};
    use paho_mqtt as mqtt;
    use rmp_serde::to_vec_named;
    use serde::{Deserialize, Serialize};

    use crate::Point2D;

    extern crate nalgebra as na;

    // A standardised "1x1" box to transform all coordinates into
    const DST_QUAD: RectCorners = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)];

    #[derive(Serialize, Deserialize, Debug)]
    pub struct TrackedPoint2D {
        id: usize,
        x: f64,
        y: f64,
    }

    /**
    clockwise: 'left top', 'right top', 'right bottom', 'left bottom',
     */
    pub type RectCorners = [Point2D; 4];
    type Matrix8x8 = na::SMatrix<f64, 8, 8>;
    pub struct PerspectiveTransformer {
        transform_matrix: Option<Matrix3<f64>>,
        output_topic: String,
    }

    impl PerspectiveTransformer {
        pub fn new(output_topic: &str, src_quad: Option<RectCorners>) -> PerspectiveTransformer {
            PerspectiveTransformer {
                transform_matrix: match src_quad {
                    Some(quad) => Some(build_transform(&quad.clone(), &DST_QUAD)),
                    None => None,
                },
                output_topic: String::from(output_topic),
            }
        }

        pub fn set_new_quad(&mut self, src_quad: &RectCorners) {
            self.transform_matrix = Some(build_transform(&src_quad, &DST_QUAD));
        }

        pub fn transform(&self, point: &Point2D) -> Result<Point2D, ()> {
            match self.transform_matrix {
                Some(matrix) => {
                    let (x, y) = point;
                    let nalgebra_point = Point2::new(*x, *y);

                    let transformed = matrix.transform_point(&nalgebra_point);
                    Ok((transformed.x, transformed.y))
                }
                None => Err(()),
            }
        }

        pub fn publish_tracked_points(
            &self,
            points: &Vec<Point2D>,
        ) -> Result<(Vec<TrackedPoint2D>, mqtt::Message), ()> {
            let points: Vec<TrackedPoint2D> = points
                .into_iter()
                .enumerate()
                .map(|i| {
                    let (index, point) = i;
                    let (x, y) = point;
                    TrackedPoint2D {
                        id: index,
                        x: *x,
                        y: *y,
                    }
                })
                .collect();
            let payload: Vec<u8> = to_vec_named(&points).unwrap();
            let message = mqtt::Message::new(&self.output_topic, payload, mqtt::QOS_1);
            Ok((points, message))
        }

        pub fn is_ready(&self) -> bool {
            match self.transform_matrix {
                Some(_) => true,
                None => false,
            }
        }
    }

    pub fn build_transform(src_quad: &RectCorners, dst_quad: &RectCorners) -> Matrix3<f64> {
        // Mappings by row - each should have 8 terms

        let r1: [f64; 8] = [
            src_quad[0].0,
            src_quad[0].1,
            1.,
            0.,
            0.,
            0.,
            -src_quad[0].0 * dst_quad[0].0,
            -src_quad[0].1 * dst_quad[0].0,
        ];
        let r2: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[0].0,
            src_quad[0].1,
            1.,
            -src_quad[0].0 * dst_quad[0].1,
            -src_quad[0].1 * dst_quad[0].1,
        ];
        let r3: [f64; 8] = [
            src_quad[1].0,
            src_quad[1].1,
            1.,
            0.,
            0.,
            0.,
            -src_quad[1].0 * dst_quad[1].0,
            -src_quad[1].1 * dst_quad[1].0,
        ];
        let r4: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[1].0,
            src_quad[1].1,
            1.,
            -src_quad[1].0 * dst_quad[1].1,
            -src_quad[1].1 * dst_quad[1].1,
        ];
        let r5: [f64; 8] = [
            src_quad[2].0,
            src_quad[2].1,
            1.,
            0.,
            0.,
            0.,
            -src_quad[2].0 * dst_quad[2].0,
            -src_quad[2].1 * dst_quad[2].0,
        ];
        let r6: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[2].0,
            src_quad[2].1,
            1.,
            -src_quad[2].0 * dst_quad[2].1,
            -src_quad[2].1 * dst_quad[2].1,
        ];
        let r7: [f64; 8] = [
            src_quad[3].0,
            src_quad[3].1,
            1.,
            0.,
            0.,
            0.,
            -src_quad[3].0 * dst_quad[3].0,
            -src_quad[3].1 * dst_quad[3].0,
        ];
        let r8: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[3].0,
            src_quad[3].1,
            1.,
            -src_quad[3].0 * dst_quad[3].1,
            -src_quad[3].1 * dst_quad[3].1,
        ];
        let combined = vec![r1, r2, r3, r4, r5, r6, r7, r8].into_iter().flatten();

        let matrix_a = Matrix8x8::from_iterator(combined);

        let dst_quad_elements = vec![
            dst_quad[0].0,
            dst_quad[0].1,
            dst_quad[1].0,
            dst_quad[1].1,
            dst_quad[2].0,
            dst_quad[2].1,
            dst_quad[3].0,
            dst_quad[3].1,
        ]
        .into_iter();

        // let matrix_b: na::SMatrix<f64, 1, 8> = na::SMatrix::from_iterator(dst_quad_elements);
        let matrix_b: na::SMatrix<f64, 1, 8> = na::SMatrix::from_iterator(dst_quad_elements);

        // Solve for Ah = B
        let coefficients = matrix_b * matrix_a.try_inverse().unwrap();
        //
        // Create a new 3x3 transform matrix using the elements from above
        let transformation_matrix = Matrix3::new(
            coefficients[0],
            coefficients[1],
            coefficients[2],
            coefficients[3],
            coefficients[4],
            coefficients[5],
            coefficients[6],
            coefficients[7],
            1.,
        );

        transformation_matrix
    }
}

#[cfg(test)]
mod tests {
    use crate::tracking::tracking::build_transform;

    use super::tracking::RectCorners;

    #[test]
    fn test_get_transform_matrix() {
        // numbers as per https://github.com/jlouthan/perspective-transform#basic-usage

        let src_quad = [158., 64., 494., 69., 495., 404., 158., 404.];
        let src_quad: RectCorners = [
            (src_quad[0], src_quad[1]),
            (src_quad[2], src_quad[3]),
            (src_quad[4], src_quad[5]),
            (src_quad[6], src_quad[7]),
        ];

        let dst_quad = [100., 500., 152., 564., 148., 604., 100., 560.];
        let dst_quad: RectCorners = [
            (dst_quad[0], dst_quad[1]),
            (dst_quad[2], dst_quad[3]),
            (dst_quad[4], dst_quad[5]),
            (dst_quad[6], dst_quad[7]),
        ];

        let transform_matrix = build_transform(&src_quad, &dst_quad);

        let src_point = (250., 120.);

        let result = {
            let (x, y) = (src_point.0, src_point.1);
            let nalgebra_point = nalgebra::Point2::new(x, y);

            let transformed = transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
        };

        assert_eq!(
            (result.0.round(), result.1.round()),
            (
                117.27521125839255_f64.round(),
                530.9202410878403_f64.round(),
            ),
        );
    }

    #[test]
    fn test_get_transform_matrix_simple() {
        // numbers as per https://github.com/jlouthan/perspective-transform#basic-usage

        let src_quad: RectCorners = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)];
        let dst_quad: RectCorners = [(1., 2.), (1., 4.), (3., 4.), (3., 2.)];

        let transform_matrix = build_transform(&src_quad, &dst_quad);

        // The transform matrix is a little different to the example in https://blog.mbedded.ninja/mathematics/geometry/projective-transformations/
        // because their point order is somehow different. The result is the same.
        // assert_eq!(
        //     transform_matrix,
        //     Matrix3::new(2., 0., 1., 0., 2., 2., 0., 0., 1.)
        // );

        let src_point = (0.5, 0.5);

        let result = {
            let (x, y) = (src_point.0, src_point.1);
            let nalgebra_point = nalgebra::Point2::new(x, y);

            let transformed = transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
        };

        assert_eq!(result, (2., 3.));
    }
}
