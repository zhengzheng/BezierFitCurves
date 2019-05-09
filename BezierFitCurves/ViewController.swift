//
//  ViewController.swift
//  BezierFitCurves
//
//  Created by zhengzheng on 2019/5/8.
//  Copyright © 2019 zhengzheng. All rights reserved.
//

import UIKit

class ViewController: UIViewController {
    let curve = UIBezierPath()
    let shapeLayer = CAShapeLayer()
    var pointViewArray = [PointView]()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        
        let slider = UISlider()
        slider.translatesAutoresizingMaskIntoConstraints = false
        slider.minimumValue = 0
        slider.maximumValue = 5000
        slider.value = 0.7
        
        slider.addTarget(self, action: #selector(sliderValueChanged(slider:)), for: .valueChanged)
        view.addSubview(slider)
        slider.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20.0).isActive = true
        slider.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20.0).isActive = true
        slider.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 40.0).isActive = true
        slider.heightAnchor.constraint(equalToConstant: 6).isActive = true
        
        var pointArray = [CGPoint]()
        let width: CGFloat = 10
        let height: CGFloat = 10
        for i in 0 ..< 6 {
            let x: CGFloat = CGFloat(i) * 60 + 30
            let y: CGFloat = 420
            let pointView = PointView.init(frame: CGRect(x: x, y: y, width: width, height: height))
            let center = CGPoint(x: x, y: y)
            pointView.center = center
            pointView.dragCallBack = { [unowned self] (pointView: PointView) -> Void in
                self.sliderValueChanged(slider: slider)
            }
            view.addSubview(pointView)
            pointViewArray.append(pointView)
            pointArray.append(center)
        }
        curve.addBezierFitCurve(pointArray)
        shapeLayer.strokeColor = UIColor.blue.cgColor
        shapeLayer.fillColor = nil
        shapeLayer.lineWidth = 3
        shapeLayer.path = curve.cgPath
        view.layer.addSublayer(shapeLayer)
    }

    @objc func sliderValueChanged(slider: UISlider) {
        curve.removeAllPoints()
        curve.error = slider.value
        
        var pointArray = [CGPoint]()
        
        for pointView in pointViewArray {
            pointArray.append(pointView.center)
        }
        curve.addBezierFitCurve(pointArray)
        shapeLayer.path = curve.cgPath
    }
}

