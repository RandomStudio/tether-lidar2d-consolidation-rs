pub mod tracking {
    use na::{Matrix2, Matrix3, Matrix4, Point2, Point3, Vector3};

    use crate::Point2D;

    extern crate nalgebra as na;
    pub struct PointXY {
        pub x: f64,
        pub y: f64,
    }

    // 'left top', 'left bottom', 'right top', 'right bottom'
    pub type RectCorners = [PointXY; 4];
    type Matrix3x3 = na::SMatrix<f64, 3, 3>;
    type Matrix4x4 = na::SMatrix<f64, 4, 4>;
    type Matrix8x8 = na::SMatrix<f64, 8, 8>;
    pub struct PerspectiveTransformer {
        transform_matrix: Matrix3<f64>,
    }

    impl PerspectiveTransformer {
        pub fn new(src_quad: &RectCorners) -> PerspectiveTransformer {
            // A standardised "1x1" box to transform all coordinates into
            let dst_quad: RectCorners = [
                PointXY { x: 0., y: 0. },
                PointXY { x: 0., y: 1. },
                PointXY { x: 1., y: 0. },
                PointXY { x: 1., y: 1. },
            ];
            PerspectiveTransformer {
                transform_matrix: build_transform(&src_quad, &dst_quad),
            }
        }

        pub fn transform(&self, point: Point2D) -> Point2D {
            let (x, y) = point;
            let nalgebra_point = Point2::new(x, y);
            let transformed = self.transform_matrix.transform_point(&nalgebra_point);
            (transformed.x, transformed.y)
        }
    }

    fn build_transform(src_quad: &RectCorners, dst_quad: &RectCorners) -> Matrix3<f64> {
        // Mappings by row - each should have 8 terms

        let r1: [f64; 8] = [
            src_quad[0].x,
            src_quad[0].y,
            1.,
            0.,
            0.,
            0.,
            -dst_quad[0].x * src_quad[0].x,
            -dst_quad[0].x * src_quad[0].y,
        ];
        let r2: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[0].x,
            src_quad[0].y,
            1.,
            -dst_quad[0].y * src_quad[0].x,
            -dst_quad[0].y * src_quad[0].y,
        ];
        let r3: [f64; 8] = [
            src_quad[1].x,
            src_quad[1].y,
            1.,
            0.,
            0.,
            0.,
            -dst_quad[1].x * src_quad[1].x,
            -dst_quad[1].x * src_quad[1].y,
        ];
        let r4: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[1].x,
            src_quad[1].y,
            1.,
            -dst_quad[1].y * src_quad[1].x,
            -dst_quad[1].y * src_quad[1].y,
        ];
        let r5: [f64; 8] = [
            src_quad[2].x,
            src_quad[2].y,
            1.,
            0.,
            0.,
            0.,
            -dst_quad[2].x * src_quad[2].x,
            -dst_quad[2].x * src_quad[2].y,
        ];
        let r6: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[2].x,
            src_quad[2].y,
            1.,
            -dst_quad[2].y * src_quad[2].x,
            -dst_quad[2].y * src_quad[2].y,
        ];
        let r7: [f64; 8] = [
            src_quad[3].x,
            src_quad[3].y,
            1.,
            0.,
            0.,
            0.,
            -dst_quad[3].x * src_quad[3].x,
            -dst_quad[3].x * src_quad[3].y,
        ];
        let r8: [f64; 8] = [
            0.,
            0.,
            0.,
            src_quad[3].x,
            src_quad[3].y,
            1.,
            -dst_quad[3].y * src_quad[3].x,
            -dst_quad[3].y * src_quad[3].y,
        ];
        let combined = vec![r1, r2, r3, r4, r5, r6, r7, r8].into_iter().flatten();

        let matrix_A = Matrix8x8::from_iterator(combined);

        let dst_quad_elements = vec![
            src_quad[0].x,
            src_quad[0].y,
            src_quad[1].x,
            src_quad[1].y,
            src_quad[2].x,
            src_quad[2].y,
            src_quad[3].x,
            src_quad[3].y,
        ]
        .into_iter();

        let matrix_B: na::SMatrix<f64, 1, 8> = na::SMatrix::from_iterator(dst_quad_elements);

        // Solve for Ah = B
        let decomp = matrix_A.lu();
        let matrix_h = decomp.solve(&matrix_B.transpose()).unwrap();

        // Create a new 3x3 transform matrix using the elements from above
        let matrix_H = Matrix3::new(
            matrix_h[0],
            matrix_h[1],
            matrix_h[2],
            matrix_h[3],
            matrix_h[4],
            matrix_h[5],
            matrix_h[6],
            matrix_h[7],
            1.,
        );

        matrix_H
    }
}
