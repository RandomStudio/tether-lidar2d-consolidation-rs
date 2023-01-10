pub mod tracking {
    use na::{Matrix3, Point2};
    use paho_mqtt as mqtt;
    use rmp_serde::to_vec_named;
    use serde::{Deserialize, Serialize};

    use crate::Point2D;

    extern crate nalgebra as na;
    #[derive(Debug)]
    pub struct PointXY {
        pub x: f64,
        pub y: f64,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct TrackedPoint2D {
        id: usize,
        x: f64,
        y: f64,
    }

    /**
    clockwise: 'left top', 'right top', 'right bottom', 'left bottom',
     */
    pub type RectCorners = [PointXY; 4];
    type Matrix8x8 = na::SMatrix<f64, 8, 8>;
    pub struct PerspectiveTransformer {
        transform_matrix: Matrix3<f64>,
        output_topic: String,
    }

    impl PerspectiveTransformer {
        pub fn new(src_quad: &RectCorners, output_topic: &str) -> PerspectiveTransformer {
            // A standardised "1x1" box to transform all coordinates into
            let dst_quad: RectCorners = [
                PointXY { x: 0., y: 0. },
                PointXY { x: 1., y: 0. },
                PointXY { x: 1., y: 1. },
                PointXY { x: 0., y: 1. },
            ];
            PerspectiveTransformer {
                transform_matrix: build_transform(&src_quad, &dst_quad),
                output_topic: String::from(output_topic),
            }
        }

        pub fn transform(&self, point: &Point2D) -> Point2D {
            let (x, y) = point;
            let nalgebra_point = Point2::new(*x, *y);

            let transformed = self.transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
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
    }

    pub fn build_transform(src_quad: &RectCorners, dst_quad: &RectCorners) -> Matrix3<f64> {
        // Mappings by row - each should have 8 terms

        let r1: [f64; 8] = [
            src_quad[0].x,
            src_quad[0].y,
            1.,
            0.,
            0.,
            0.,
            -src_quad[0].x * dst_quad[0].x,
            -src_quad[0].y * dst_quad[0].x,
        ];
        let r2: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[0].x,
            src_quad[0].y,
            1.,
            -src_quad[0].x * dst_quad[0].y,
            -src_quad[0].y * dst_quad[0].y,
        ];
        let r3: [f64; 8] = [
            src_quad[1].x,
            src_quad[1].y,
            1.,
            0.,
            0.,
            0.,
            -src_quad[1].x * dst_quad[1].x,
            -src_quad[1].y * dst_quad[1].x,
        ];
        let r4: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[1].x,
            src_quad[1].y,
            1.,
            -src_quad[1].x * dst_quad[1].y,
            -src_quad[1].y * dst_quad[1].y,
        ];
        let r5: [f64; 8] = [
            src_quad[2].x,
            src_quad[2].y,
            1.,
            0.,
            0.,
            0.,
            -src_quad[2].x * dst_quad[2].x,
            -src_quad[2].y * dst_quad[2].x,
        ];
        let r6: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[2].x,
            src_quad[2].y,
            1.,
            -src_quad[2].x * dst_quad[2].y,
            -src_quad[2].y * dst_quad[2].y,
        ];
        let r7: [f64; 8] = [
            src_quad[3].x,
            src_quad[3].y,
            1.,
            0.,
            0.,
            0.,
            -src_quad[3].x * dst_quad[3].x,
            -src_quad[3].y * dst_quad[3].x,
        ];
        let r8: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[3].x,
            src_quad[3].y,
            1.,
            -src_quad[3].x * dst_quad[3].y,
            -src_quad[3].y * dst_quad[3].y,
        ];
        let combined = vec![r1, r2, r3, r4, r5, r6, r7, r8].into_iter().flatten();

        let matrix_a = Matrix8x8::from_iterator(combined);

        let dst_quad_elements = vec![
            dst_quad[0].x,
            dst_quad[0].y,
            dst_quad[1].x,
            dst_quad[1].y,
            dst_quad[2].x,
            dst_quad[2].y,
            dst_quad[3].x,
            dst_quad[3].y,
        ]
        .into_iter();

        let matrix_b: na::SMatrix<f64, 1, 8> = na::SMatrix::from_iterator(dst_quad_elements);

        // Solve for Ah = B
        // let decomp = matrix_a.lu();
        // let coefficients = decomp.solve(&matrix_b.transpose()).unwrap();
        let coefficients = matrix_b * matrix_a.try_inverse().unwrap();

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
    use nalgebra::{Matrix3, Point};
    extern crate approx;

    use crate::tracking::tracking::build_transform;

    use super::tracking::{PerspectiveTransformer, PointXY, RectCorners};

    // #[test]
    // fn test_get_transform_matrix() {
    //     // numbers as per https://github.com/jlouthan/perspective-transform#basic-usage

    //     let src_quad = [158., 64., 494., 69., 495., 404., 158., 404.];
    //     let src_quad: RectCorners = [
    //         PointXY {
    //             x: src_quad[0],
    //             y: src_quad[1],
    //         },
    //         PointXY {
    //             x: src_quad[2],
    //             y: src_quad[3],
    //         },
    //         PointXY {
    //             x: src_quad[4],
    //             y: src_quad[5],
    //         },
    //         PointXY {
    //             x: src_quad[6],
    //             y: src_quad[7],
    //         },
    //     ];

    //     let dst_quad = [100., 500., 152., 564., 148., 604., 100., 560.];
    //     let dst_quad: RectCorners = [
    //         PointXY {
    //             x: dst_quad[0],
    //             y: dst_quad[1],
    //         },
    //         PointXY {
    //             x: dst_quad[2],
    //             y: dst_quad[3],
    //         },
    //         PointXY {
    //             x: dst_quad[4],
    //             y: dst_quad[5],
    //         },
    //         PointXY {
    //             x: dst_quad[6],
    //             y: dst_quad[7],
    //         },
    //     ];

    //     let transform_matrix = build_transform(&src_quad, &dst_quad);

    //     let src_point = PointXY { x: 250., y: 120. };

    //     let result = {
    //         let (x, y) = (src_point.x, src_point.y);
    //         let nalgebra_point = nalgebra::Point2::new(x, y);

    //         let transformed = transform_matrix.transform_point(&nalgebra_point);
    //         (transformed.x, transformed.y)
    //     };

    //     assert_eq!(result, (117.27521125839255, 530.9202410878403));
    // }

    #[test]
    fn test_get_transform_matrix_simple() {
        // numbers as per https://github.com/jlouthan/perspective-transform#basic-usage

        let src_quad: RectCorners = [
            PointXY { x: 0., y: 0. },
            PointXY { x: 1., y: 0. },
            PointXY { x: 1., y: 1. },
            PointXY { x: 0., y: 1. },
        ];
        let dst_quad: RectCorners = [
            PointXY { x: 1., y: 2. },
            PointXY { x: 1., y: 4. },
            PointXY { x: 3., y: 4. },
            PointXY { x: 3., y: 2. },
        ];

        let transform_matrix = build_transform(&src_quad, &dst_quad);

        assert_eq!(
            transform_matrix,
            Matrix3::new(2., 0., 1., 0., 2., 2., 0., 0., 1.)
        );

        let src_point = PointXY { x: 0.5, y: 0.5 };

        let result = {
            let (x, y) = (src_point.x, src_point.y);
            let nalgebra_point = nalgebra::Point2::new(x, y);

            let transformed = transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
        };

        assert_eq!(result, (2., 3.));
    }
}
