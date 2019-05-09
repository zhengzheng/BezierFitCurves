//
//  PointView.swift
//  BezierFitCurves
//
//  Created by zhengzheng on 2019/5/9.
//  Copyright © 2019 zhengzheng. All rights reserved.
//

import UIKit

class PointView: UIControl {
    var dragCallBack = { (pointView: PointView) -> Void in }
    
    override init(frame: CGRect) {
        super.init(frame: frame)
        layer.cornerRadius = 5
        layer.masksToBounds = true
        backgroundColor = UIColor.magenta
        self.addTarget(self, action: #selector(dragInside(pointView:withEvent:)), for: .touchDragInside)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    @objc func dragInside(pointView: PointView, withEvent event: UIEvent) {
        if let touches = event.allTouches {
            for touch in touches {
                self.center = touch.location(in: superview)
                dragCallBack(self)
                return
            }
        }
    }
}
