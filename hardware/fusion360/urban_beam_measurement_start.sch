<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
  <drawing>
    <settings>
      <setting alwaysvectorfont="yes"/>
      <setting verticaltext="up"/>
    </settings>
    <grid distance="2.54" unitdist="mm" unit="mm" style="lines" multiple="1" display="yes" altdistance="1.27" altunitdist="mm" altunit="mm"/>
    <layers>
      <layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
      <layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
      <layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
      <layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
      <layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
    </layers>
    <schematic xreflabel="%F%N/%S.%C%R">
      <libraries/>
      <attributes/>
      <variantdefs/>
      <classes>
        <class number="0" name="default" width="0" drill="0"/>
      </classes>
      <parts/>
      <sheets>
        <sheet>
          <plain>
            <text x="10" y="190" size="2.54" layer="91">URBAN BEAM - SWR measurement board (starter schematic)</text>
            <text x="10" y="184" size="1.778" layer="91">Blocks to instantiate: directional coupler, 2x AD8307, ADC filters, 5V->3V3 LDO, connector to ESP32-S3.</text>
            <text x="10" y="176" size="1.778" layer="91">Nets to preserve: +5V, +3V3A, GND, ADC_FWD, ADC_REV, RF_FWD_IN, RF_REV_IN.</text>
          </plain>
          <instances/>
          <busses/>
          <nets/>
        </sheet>
      </sheets>
    </schematic>
  </drawing>
</eagle>
