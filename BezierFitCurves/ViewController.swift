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
    let slider = UISlider()
    let errorLabel = UILabel(frame: CGRect.zero)
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        
        slider.translatesAutoresizingMaskIntoConstraints = false
        slider.minimumValue = 0
        slider.maximumValue = 5000
        slider.value = 1000
        
        slider.addTarget(self, action: #selector(sliderValueChanged), for: .valueChanged)
        view.addSubview(slider)
        slider.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20.0).isActive = true
        slider.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20.0).isActive = true
        slider.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 40.0).isActive = true
        slider.heightAnchor.constraint(equalToConstant: 6).isActive = true
        
        
        let button = UIButton(type: .system)
        button.setTitle("Generate", for: .normal)
        button.translatesAutoresizingMaskIntoConstraints = false
        button.addTarget(self, action: #selector(sliderValueChanged), for: .touchUpInside)
        view.addSubview(button)
        
        button.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20.0).isActive = true
        button.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 70.0).isActive = true
        
        errorLabel.text = String(slider.value)
        errorLabel.font = UIFont.systemFont(ofSize: 14)
        errorLabel.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(errorLabel)
        
        errorLabel.centerYAnchor.constraint(equalTo: button.centerYAnchor).isActive = true
        errorLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor).isActive = true
        
        
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
                self.sliderValueChanged()
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

    @objc func sliderValueChanged() {
        errorLabel.text = String(self.slider.value)
        curve.removeAllPoints()
        curve.error = self.slider.value
        
        var pointArray = [CGPoint]()
        
        for pointView in pointViewArray {
            pointArray.append(pointView.center)
        }
        curve.addBezierFitCurve(pointArray)
        shapeLayer.path = curve.cgPath
    }
}

