//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 368 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CPositionalLightH
#define CPositionalLightH
//---------------------------------------------------------------------------
#include "lighting/CGenericLight.h"
//--------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CPositionalLight.h

    \brief 
    <b> Lighting </b> \n 
    Positional Light Source.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cPositionalLight
    \ingroup    lighting

    \brief      
    cPositionalLight describes a positional light source. 
*/
//===========================================================================

class cPositionalLight : public virtual cGenericLight
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cPositionalLight.
    cPositionalLight(cWorld* a_world);
	
    //! Destructor of cPositionalLight.
    virtual ~cPositionalLight();


	//-----------------------------------------------------------------------
    // METHODS: (lighting properties)
	//-----------------------------------------------------------------------

    //! Set my constant attenuation parameter.
    void setAttConstant(const GLfloat& a_value) { m_attConstant = cClamp(a_value, 0.0f, 1.0f); }

    //! Read my constant attenuation parameter.
    GLfloat getAttConstant() const { return (m_attConstant); }

    //! Set my linear attenuation parameter.
    void setAttLinear(const GLfloat& a_value) { m_attLinear = cClamp(a_value, 0.0f, 1.0f); }

    //! Read my linear attenuation parameter.
    GLfloat getAttLinear() const { return (m_attLinear); }

    //! Set my quadratic attenuation parameter.
    void setAttQuadratic(const GLfloat& a_value) { m_attQuadratic = cClamp(a_value, 0.0f, 1.0f); }

    //! Read my quadratic attenuation parameter.
    GLfloat getAttQuadratic() const { return (m_attQuadratic); }


	//-----------------------------------------------------------------------
    // METHODS: (display model)
	//-----------------------------------------------------------------------

    //! Set display settings of light source. To be used for debugging purposes to display location of light source.
    void setDisplaySettings(const double& a_sourceRadius = 0.02,
                            const bool a_displayEnabled = true);


  	//-----------------------------------------------------------------------
    // MEMBERS: (display model)
	//-----------------------------------------------------------------------

    //! Color used for rendering the display model of the light source. 
    cColorf m_displaySourceColor;


  protected:
  
  	//-----------------------------------------------------------------------
    // MEMBERS: (lighting properties)
	//-----------------------------------------------------------------------

	//! Constant attenuation parameter.
    GLfloat m_attConstant;
    
	//! Linear attenuation parameter.
    GLfloat m_attLinear;
    
	//! Quadratic attenuation parameter.
    GLfloat m_attQuadratic;


  	//-----------------------------------------------------------------------
    // MEMBERS: (display model)
	//-----------------------------------------------------------------------

    //! Radius of display model representating light source.
    double m_displaySourceRadius;
    

	//-----------------------------------------------------------------------
	// METHODS:
	//-----------------------------------------------------------------------

    //! Render the lighting properties of this light source in OpenGL.
    virtual void renderLightSource(cRenderOptions& a_options);

    //! Render a graphical representation (display model) of the light source. (used for debuging purposes typically).
    virtual void render(cRenderOptions& a_options);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

