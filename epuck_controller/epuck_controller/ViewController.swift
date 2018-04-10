//
//  ViewController.swift
//  epuck_controller
//
//  Created by Nicolas Peslerbe on 09.04.18.
//  Copyright © 2018 Nicolas Peslerbe. All rights reserved.
//

import UIKit
import AudioKit


class ViewController: UIViewController {
    let oscillator = AKOscillator()
    var envelope = AKAmplitudeEnvelope()

    var playing:Bool = false
    @IBAction func discoverClicked(_ sender: Any) {
        if playing{return}
        oscillator.frequency = 250
        playSong(oscillator)
    }
    
    @IBAction func exploreClicked(_ sender: Any) {
        if playing{return}
        oscillator.frequency = 296
        playSong(oscillator)
    }
    @IBAction func sendMapClicked(_ sender: Any) {
        if playing{return}
        oscillator.frequency = 406
        playSong(oscillator)
    }
    @IBAction func sortClicked(_ sender: Any) {
        if playing{return}
        oscillator.frequency = 359
        playSong(oscillator)
    }
    @IBAction func singClicked(_ sender: Any) {
        if playing{return}
        oscillator.frequency = 452
        playSong(oscillator)
    }
    override func viewDidLoad() {
        super.viewDidLoad()
        envelope = AKAmplitudeEnvelope(oscillator)
        envelope.attackDuration = 0.3
        envelope.decayDuration = 0.1
        envelope.sustainLevel = 0.1
        envelope.releaseDuration = 0.3
        oscillator.rampTime = 0.2
        oscillator.amplitude = 0.9
        AudioKit.output = oscillator
        do {
            try AudioKit.start()
        } catch _ {}
        // Do any additional setup after loading the view, typically from a nib.
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    func playSong(_ oscillator:AKOscillator){
        playing = true
        oscillator.start()
        sleep(3)
        oscillator.stop()
//        AudioKit.output = oscillator
//        do {
//            try AudioKit.stop()
//        } catch _ {}
        playing = false
    }
}

