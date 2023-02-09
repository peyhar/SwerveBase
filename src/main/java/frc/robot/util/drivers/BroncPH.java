// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.drivers;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;

/** Creates a pcm, singleton so it can be accessed by multiple subsystems */
public class BroncPH {

  private static BroncPH m_instance = null;
  private static PneumaticHub m_actual_pcm = null;
  private static Compressor m_compressor = null;

  public static BroncPH getInstance() {
    if (m_instance == null) {
      m_instance = new BroncPH();
      m_actual_pcm = new PneumaticHub(41);
      m_compressor = m_actual_pcm.makeCompressor();
      m_compressor.enableDigital();
    }
    return m_instance;
  }

  public PneumaticHub getPCM() {
    return m_actual_pcm;
  }

  public Compressor getCompressor() {
    return m_compressor;
  }

  private BroncPH() {}
}
