//
//  UIBezierPath+FitCurves.swift
//  Map
//
//  Created by zhengzheng on 2019/5/8.
//  Copyright © 2019 Sogou, Inc. All rights reserved.
//

import Foundation
import UIKit
import simd

struct BezierCurve {
    var pt1: simd_float2
    var pt2: simd_float2
    var c1: simd_float2
    var c2: simd_float2
}

private var _error: Float = 1000

extension UIBezierPath {
    var error: Float {
        get {
            return _error
        }
        set {
            _error = max(0, newValue)
        }
    }
    
    func addBezierFitCurve(_ points: [CGPoint]) {
        let count = points.count
        if count <= 1 {
            return
        }
        else if count <= 2 {
            move(to: points[0])
            switch count {
            case 2:
                addLine(to: points[1])
            default:
                break
            }
            return
        }

        let d = points.map { p in
            simd_float2(Float(p.x), Float(p.y))
        }
        let bezCurves = FitCurve(d, _error)
        move(to: points[0])
        for bezCurve in bezCurves {
            let _ = CGPoint(x: CGFloat(bezCurve.pt1.x), y: CGFloat(bezCurve.pt1.y))
            let pt2 = CGPoint(x: CGFloat(bezCurve.pt2.x), y: CGFloat(bezCurve.pt2.y))
            let c1 = CGPoint(x: CGFloat(bezCurve.c1.x), y: CGFloat(bezCurve.c1.y))
            let c2 = CGPoint(x: CGFloat(bezCurve.c2.x), y: CGFloat(bezCurve.c2.y))
            addCurve(to: pt2, controlPoint1: c1, controlPoint2: c2)
        }
    }
}

/*
 *  FitCurve :
 *      Fit a Bezier curve to a set of points.
 */
func FitCurve(_ d: [simd_float2], _ error: Float) -> [BezierCurve] {
    // Unit tangent vectors at endpoints.
    var tHat1: simd_float2
    var tHat2: simd_float2
    // The vector that will store the BezierCurve
    var bezCurves = [BezierCurve]()
    
    let startIndex = 0
    let endIndex = d.count
    // if startIndex is the beginning of the curve.
    tHat1 = computeLeftTangent(d, startIndex);
    // if endIndex is the end of the curve.
    tHat2 = computeRightTangent(d, endIndex - 1);
    FitCubic(d, &bezCurves, startIndex, endIndex - 1, tHat1, tHat2, error)
    return bezCurves
}

func addBezierCurve(_ bezCurve: [simd_float2], _ bezCurves: inout [BezierCurve]) {
    let newBezier = BezierCurve(pt1: bezCurve[0], pt2: bezCurve[3], c1: bezCurve[1], c2: bezCurve[2])
    bezCurves.append(newBezier)
}

/*
 *  FitCubic :
 *      Fit a Bezier curve to a (sub)set of points
 */
func FitCubic(_ d: [simd_float2], _ bezCurves: inout [BezierCurve],
              _ first: Int, _ last: Int, _ tHat1: simd_float2, _ tHat2: simd_float2, _ error: Float) {
    // Control points of fitted Bezier curve;
    var bezCurve = [simd_float2](repeating: simd_float2(), count: 4)
    // Parameter values for point
    var u = [Float](repeating: 0.0, count: last - first + 1)
    // Improved parameter values
    var uPrime = [Float](repeating: 0.0, count: last - first + 1)
    
    var maxError: Float         // Maximum fitting error
    var splitPoint: Int = 0     // Point to split point set at
//    var nPts: Int               // Number of points in subset
//    var iterationError: Float   // Error below which you try iterating
    let maxIterations: Int = 20 // Max times to try iterating
//    var tHatCenter: simd_float2 // Unit tangent vector at splitPoint
    
    // Error below which you try iterating
    let iterationError = error * error
    // Number of points in subset
    let nPts = last - first + 1
    if nPts == 1 {
        print("Only have 1 point, so no fitting")
        return
    }
    
    //  Use heuristic if region only has two points in it
    if nPts == 2 {
        let dist = simd_distance(d[last], d[first]) / 3.0
        bezCurve[0] = d[first];
        bezCurve[3] = d[last];
        bezCurve[1] = bezCurve[0] + scaleVec(tHat1, dist)
        bezCurve[2] = bezCurve[3] + scaleVec(tHat2, dist)
        addBezierCurve(bezCurve, &bezCurves)
        print("Fit 2 Points, use heuristic")
        return
    }
    // Parameterize points, and attempt to fit curve
    chordLengthParameterize(d, first, last, &u)
    generateBezier(d, &bezCurve, first, last, u, tHat1, tHat2)
    
    //  Find max deviation of points to fitted curve
    maxError = computeMaxError(d, first, last, &bezCurve, &u, &splitPoint)
    print("maxError = \(maxError), splitPoint = \(splitPoint)")

    if (maxError < error) {
        addBezierCurve(bezCurve, &bezCurves)
        return;
    }
    //  If error not too large, try some reparameterization
    //  and iteration
    if maxError < iterationError {
        for _ in 0 ..< maxIterations {
            uPrime = reparameterize(d, first, last, u, bezCurve)
            generateBezier(d, &bezCurve, first, last, uPrime, tHat1, tHat2)
            maxError = computeMaxError(d, first, last, &bezCurve, &uPrime, &splitPoint)
            if (maxError < error) {
                addBezierCurve(bezCurve, &bezCurves)
                return
            }
            u = uPrime
        }
    }
    // Fitting failed -- split at max error point and fit recursively
    let tHatCenter = computeCenterTangent(d, splitPoint)    // Unit tangent vector at splitPoint
    FitCubic(d, &bezCurves, first, splitPoint, tHat1, tHatCenter, error)
    FitCubic(d, &bezCurves, splitPoint, last, -tHatCenter, tHat2, error)
}

func generateBezier(_ d: [simd_float2], _ bezCurve: inout [simd_float2],
                    _ first: Int, _ last: Int, _ uPrime: [Float], _ tHat1: simd_float2, _ tHat2: simd_float2) {
    let MAXPOINTS: Int = 1000
    // cv::Vec2d A[MAXPOINTS][2];  // Precomputed rhs for eqn
    var A = [[simd_float2]](repeating: [simd_float2](repeating: simd_float2(), count: 2), count: MAXPOINTS)
    let nPts = last - first + 1 // Number of pts in sub-curve
    var C = [[Float]](repeating: [Float](repeating: 0.0, count: 2), count: 2)   // Matrix C
    var X = [Float](repeating: 0.0, count: 2)   // Matrix X
//    var det_C0_C1, det_C0_X, det_X_C1: Float    // Determinants of matrices
//    var alpha_l, alpha_r: Float                 // Alpha values, left and right
    var tmp: simd_float2                        // Utility variable
    
    // Compute the A
    for i in 0 ..< nPts {
        let v1 = scaleVec(tHat1, B1(uPrime[i]))
        let v2 = scaleVec(tHat2, B2(uPrime[i]))
        A[i][0] = v1
        A[i][1] = v2
    }
    for i in 0 ..< nPts {
        C[0][0] += simd_dot(A[i][0], A[i][0])
        C[0][1] += simd_dot(A[i][0], A[i][1])
        
        C[1][0] = C[0][1]
        
        C[1][1] += simd_dot(A[i][1], A[i][1])
        
        let b0 = B0(uPrime[i])
        let b1 = B1(uPrime[i])
        let b2 = B2(uPrime[i])
        let b3 = B3(uPrime[i])
        
        let d0 = d[first + i]
        let d1 = d[first] * b0
        let d2 = d[first] * b1
        let d3 = d[last] * b2
        let d4 = d[last] * b3
        tmp = d0 - (d1 + (d2 + (d3 + d4)))
        
        X[0] += simd_dot(A[i][0], tmp)
        X[1] += simd_dot(A[i][1], tmp)
    }
    
    // Compute the determinants of C and X
    let det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1]
    let det_C0_X = C[0][0] * X[1] - C[1][0] * X[0]
    let det_X_C1 = X[0] * C[1][1] - X[1] * C[0][1]
    
    // Finally, derive alpha values
    let alpha_l = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1
    let alpha_r = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1
    
    // Checks for "dangerous" points, meaning that the alpha_l or alpha_r are abnormally large
    // from here http://newsgroups.derkeiler.com/Archive/Comp/comp.graphics.algorithms/2005-08/msg00419.html
    // This is a common problem with this algoithm.
    let dif1 = simd_distance(d[first], d[last])
    var danger = false
    if (alpha_l > dif1 * 2) || (alpha_r > dif1 * 2) {
//        first += 0
        danger = true
    }
    
    //  If alpha negative, use the Wu/Barsky heuristic (see text)
    //  (if alpha is 0, you get coincident control points that lead to
    //  divide by zero in any subsequent NewtonRaphsonRootFind() call.
    let segLength = simd_distance(d[first], d[last])
    let epsilon = 1.0e-6 * segLength
    
    if alpha_l < epsilon || alpha_r < epsilon || danger {
        // fall back on standard (probably inaccurate) formula, and subdivide further if needed.
        let dist = segLength / 3.0
        bezCurve[0] = d[first]
        bezCurve[3] = d[last]
        bezCurve[1] = bezCurve[0] + scaleVec(tHat1, dist)
        bezCurve[2] = bezCurve[3] + scaleVec(tHat2, dist)
        return  //bezCurve;
    }
    
    //  First and last control points of the Bezier curve are
    //  positioned exactly at the first and last data points
    //  Control points 1 and 2 are positioned an alpha distance out
    //  on the tangent vectors, left and right, respectively
    bezCurve[0] = d[first];
    bezCurve[3] = d[last];
    bezCurve[1] = bezCurve[0] + scaleVec(tHat1, alpha_l)
    bezCurve[2] = bezCurve[3] + scaleVec(tHat2, alpha_r)
}
/*
 *  Reparameterize:
 *    Given set of points and their parameterization, try to find
 *   a better parameterization.
 */
func reparameterize(_ d: [simd_float2], _ first: Int, _ last: Int, _ u: [Float], _ bezCurve: [simd_float2]) -> [Float] {
    var uPrime = [Float]()
    for i in first ... last {
        let tmp = newtonRaphsonRootFind(bezCurve, d[i], u[i - first])
        uPrime.append(tmp)
    }
    return uPrime
}

/*
 *  NewtonRaphsonRootFind :
 *    Use Newton-Raphson iteration to find better root.
 */
func newtonRaphsonRootFind(_ Q: [simd_float2], _ P: simd_float2, _ u: Float) -> Float {
//    var numerator, denominator: Float
    //  Q' and Q''
    var Q1 = [simd_float2]()
    var Q2 = [simd_float2]()
//    var Q_u, Q1_u, Q2_u: simd_float2; // u evaluated at Q, Q', & Q''
//    var uPrime: Float   // Improved u
    
    // Compute Q(u)
    let Q_u = bezierII(3, Q, u);
    
    // Generate control vertices for Q'
    for i in 0 ... 2 {
        let x = (Q[i + 1].x - Q[i].x) * 3.0
        let y = (Q[i + 1].y - Q[i].y) * 3.0
        Q1.append(simd_float2(x, y))
    }
    
    // Generate control vertices for Q''
    for i in 0 ... 1 {
        let x = (Q[i + 1].x - Q[i].x) * 2.0
        let y = (Q[i + 1].y - Q[i].y) * 2.0
        Q2.append(simd_float2(x, y))
    }
    // u evaluated at Q, Q', & Q''
    // Compute Q'(u) and Q''(u)
    let Q1_u = bezierII(2, Q1, u)
    let Q2_u = bezierII(1, Q2, u)
    
    // Compute f(u)/f'(u)
    let numerator = (Q_u[0] - P.x) * (Q1_u[0]) + (Q_u[1] - P.y) * (Q1_u[1])
    let denominator = (Q1_u[0]) * (Q1_u[0]) + (Q1_u[1]) * (Q1_u[1]) +
        (Q_u[0] - P.x) * (Q2_u[0]) + (Q_u[1] - P.y) * (Q2_u[1])
    if denominator == 0.0 {
        return u
    }
    
    // u = u - f(u)/f'(u)   Improved u
    let uPrime = u - (numerator / denominator)
    return uPrime
}

/*
 *  Bezier :
 *      Evaluate a Bezier curve at a particular parameter value
 */
func bezierII(_ degree: Int, _ V: [simd_float2], _ t: Float) -> simd_float2 {
    var Q: simd_float2          // Point on curve at parameter t
    var Vtemp = [simd_float2]() // Local copy of control points
    for i in 0 ... degree {
        Vtemp.append(V[i])
    }
    // Triangle computation
    for i in 1 ... degree {
        for j in 0 ... degree - i {
            Vtemp[j].x = (1.0 - t) * Vtemp[j].x + t * Vtemp[j + 1].x;
            Vtemp[j].y = (1.0 - t) * Vtemp[j].y + t * Vtemp[j + 1].y;
        }
    }
    Q = Vtemp[0]
    return Q
}

/*
 *  B0, B1, B2, B3 :
 *    Bezier multipliers
 */
func B0(_ u: Float) -> Float {
    let tmp = 1.0 - u
    return tmp * tmp * tmp
}

func B1(_ u: Float) -> Float {
    let tmp = 1.0 - u
    return 3 * u * (tmp * tmp)
}

func B2(_ u: Float) -> Float {
    let tmp = 1.0 - u
    return 3 * u * u * tmp
}

func B3(_ u: Float) -> Float {
    return u * u * u
}

/*
 * ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent :
 * Approximate unit tangents at endpoints and "center" of the curve.
 */
func computeLeftTangent(_ d: [simd_float2], _ end: Int) -> simd_float2 {
    var tHat1: simd_float2
    if end == 0 {
        tHat1 = d[end + 1] - d[end]
    }
    else {
        tHat1 = d[end + 1] - d[end - 1]
    }
    tHat1 = simd_normalize(tHat1)
    return tHat1
}

func computeRightTangent(_ d: [simd_float2], _ end: Int) -> simd_float2 {
    var tHat2: simd_float2
    if end == d.count - 1 {
        tHat2 = d[end - 1] - d[end]
    }
    else {
        tHat2 = d[end - 1] - d[end + 1]
    }
    tHat2 = simd_normalize(tHat2)
    return tHat2
}

func computeCenterTangent(_ d: [simd_float2], _ center: Int) -> simd_float2 {
    let V1 = d[center - 1] - d[center]
    let V2 = d[center] - d[center + 1]
    var tHatCenter = simd_float2(x: (V1.x + V2.x) / 2.0, y: (V1.y + V2.y) / 2.0)
    tHatCenter = simd_normalize(tHatCenter)
    return tHatCenter
}

/*
 *  ChordLengthParameterize :
 *    Assign parameter values to points
 *    using relative distances between points.
 */
func chordLengthParameterize(_ d: [simd_float2], _ first: Int, _ last: Int, _ u: inout [Float]) {
    u[0] = 0.0
    for i in first + 1 ... last {   // for (i = first + 1; i <= last; ++i)
        u[i - first] = u[i - first - 1] + simd_distance(d[i], d[i - 1])
    }
    for i in first + 1 ... last {
        u[i - first] = u[i - first] / u[last - first]
    }
}

func computeMaxError(_ d: [simd_float2],
                     _ first: Int,
                     _ last: Int,
                     _ bezCurve: inout [simd_float2],
                     _ u: inout [Float],
                     _ splitPoint: inout Int) -> Float {
    var maxDist: Float = 0.0    // Maximum error
//    var dist: Float             // Current error
//    var P: simd_float2          // Point on curve
//    var v: simd_float2          // Vector from point to curve
    splitPoint = (last - first + 1) / 2
    for i in first + 1 ..< last {
        let P = bezierII(3, bezCurve, u[i - first])
        let v = P - d[i]
        let dist = v[0] * v[0] + v[1] * v[1]
        if dist >= maxDist {
            maxDist = dist
            splitPoint = i
        }
    }
    return maxDist
}

func scaleVec(_ v: simd_float2, _ newLen: Float) -> simd_float2 {
    let normal = simd_normalize(v)
    let ret = normal * newLen
    return ret
}
