//
// Created by shida on 06/12/16.
//

#include "Modeler/ModelDrawer.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <list>
#include <GL/gl.h>       // Core OpenGL functions
#include <GL/glu.h>      // OpenGL utility functions (optional, based on usage)


namespace ORB_SLAM2
{
    ModelDrawer::ModelDrawer():mbModelUpdateRequested(false), mbModelUpdateDone(true)
    {   
        // initialize_empty_texture_map(3000,3000);
    }
    void ModelDrawer::DrawModel(bool bRGB, vector<pair<cv::Mat, TextureFrame>> imAndTexFrame) {
        if (imAndTexFrame.empty()) return;

        int numKFs = imAndTexFrame.size();
        vector<unsigned int> frameTex(numKFs, 0);
        
        glGenTextures(numKFs, frameTex.data());
        for (size_t i = 0; i < numKFs; i++) {
            cv::Size imSize = imAndTexFrame[i].first.size();
            glBindTexture(GL_TEXTURE_2D, frameTex[i]);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            if (bRGB) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                            imSize.width, imSize.height, 0,
                            GL_BGR, GL_UNSIGNED_BYTE,
                            imAndTexFrame[i].first.data);
            } else {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                            imSize.width, imSize.height, 0,
                            GL_RGB, GL_UNSIGNED_BYTE,
                            imAndTexFrame[i].first.data);
            }
        }

        UpdateModel();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_TEXTURE_2D);

        const auto& tris = GetTris();
        vector<vector<pair<int, vector<float>>>> triangleTextures;  
        for (auto triIt = tris.begin(); triIt != tris.end(); ++triIt) {
            const auto& triangle = *triIt;
            vector<pair<int, vector<float>>> visibleTextures;

            dlovi::Matrix point0 = GetPoints()[triangle(0)];
            dlovi::Matrix point1 = GetPoints()[triangle(1)];
            dlovi::Matrix point2 = GetPoints()[triangle(2)];

            dlovi::Matrix edge10 = point1 - point0;
            dlovi::Matrix edge20 = point2 - point0;
            dlovi::Matrix normal = edge20.cross(edge10);
            normal = normal / normal.norm();

            for (int frameIdx = 0; frameIdx < numKFs; frameIdx++) {
                cv::Mat texOrient = imAndTexFrame[frameIdx].second.GetOrientation();
                dlovi::Matrix orientation(3,1);
                orientation(0) = texOrient.at<float>(0);
                orientation(1) = texOrient.at<float>(1);
                orientation(2) = texOrient.at<float>(2);

                double dotProduct = normal.dot(orientation);
                
                if (dotProduct > 0) {
                    TextureFrame tex = imAndTexFrame[frameIdx].second;
                    cv::Size imSize = imAndTexFrame[frameIdx].first.size();
                    
                    vector<float> uv0 = tex.GetTexCoordinate(point0(0), point0(1), point0(2), imSize);
                    vector<float> uv1 = tex.GetTexCoordinate(point1(0), point1(1), point1(2), imSize);
                    vector<float> uv2 = tex.GetTexCoordinate(point2(0), point2(1), point2(2), imSize);

                    if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {
                        vector<float> uvCoords;
                        uvCoords.insert(uvCoords.end(), uv0.begin(), uv0.end());
                        uvCoords.insert(uvCoords.end(), uv1.begin(), uv1.end());
                        uvCoords.insert(uvCoords.end(), uv2.begin(), uv2.end());
                        visibleTextures.push_back({frameIdx, uvCoords});
                    }
                }
            }
            triangleTextures.push_back(visibleTextures);
        }

        auto triIt = tris.begin();
        for (size_t i = 0; i < triangleTextures.size(); ++i, ++triIt) {
            const auto& triangle = *triIt;
            const auto& textures = triangleTextures[i];
            
            if (textures.empty()) continue;

            dlovi::Matrix point0 = GetPoints()[triangle(0)];
            dlovi::Matrix point1 = GetPoints()[triangle(1)];
            dlovi::Matrix point2 = GetPoints()[triangle(2)];
            dlovi::Matrix edge10 = point1 - point0;
            dlovi::Matrix edge20 = point2 - point0;
            dlovi::Matrix normal = edge20.cross(edge10);
            normal = normal / normal.norm();

            float alpha = 1.0f;

            for (const auto& tex : textures) {
                int frameIdx = tex.first;
                const vector<float>& uvCoords = tex.second;

                glBindTexture(GL_TEXTURE_2D, frameTex[frameIdx]);
                
                glBegin(GL_TRIANGLES);
                glColor4f(1.0, 1.0, 1.0, alpha);
                glNormal3d(normal(0), normal(1), normal(2));

                glTexCoord2f(uvCoords[0], uvCoords[1]);
                glVertex3d(point0(0), point0(1), point0(2));

                glTexCoord2f(uvCoords[2], uvCoords[3]);
                glVertex3d(point1(0), point1(1), point1(2));

                glTexCoord2f(uvCoords[4], uvCoords[5]);
                glVertex3d(point2(0), point2(1), point2(2));
                glEnd();
            }
        }

        glDisable(GL_BLEND);
        glDisable(GL_TEXTURE_2D);
    }

    void ModelDrawer::SaveModelToObj(const std::vector<dlovi::Matrix>& points, 
                                const std::list<dlovi::Matrix>& triangles, 
                                const std::string& filename) 
    {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
            return;
        }

        // Write vertices
        for (const auto& point : points) {
            file << "v " << point(0) << " " << point(1) << " " << point(2) << "\n";
        }

        // Write faces (triangles)
        for (const auto& tri : triangles) {
            file << "f " 
                << static_cast<int>(tri(0) + 1) << " "
                << static_cast<int>(tri(1) + 1) << " "
                << static_cast<int>(tri(2) + 1) << "\n";
        }

        file.close();
        std::cout << "Model saved to " << filename << std::endl;
    }


//     void ModelDrawer::DrawModel(bool bRGB, vector<pair<cv::Mat,TextureFrame>> imAndTexFrame)
//     {
//         int numKFs = 1;
//         if (imAndTexFrame.size() >= numKFs) {
//             static unsigned int frameTex[1] = {0};
//             if (!frameTex[0])
//                 glGenTextures(numKFs, frameTex);

//             cv::Size imSize = imAndTexFrame[0].first.size();

//             for (int i = 0; i < numKFs; i++) {
//                 glBindTexture(GL_TEXTURE_2D, frameTex[i]);
//                 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//                 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//                 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//                 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//                 // image are saved in RGB format, grayscale images are converted
//                 if (bRGB) {
//                     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
//                                  imSize.width, imSize.height, 0,
//                                  GL_BGR,
//                                  GL_UNSIGNED_BYTE,
//                                  imAndTexFrame[i].first.data);
//                 } else {
//                     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
//                                  imSize.width, imSize.height, 0,
//                                  GL_RGB,
//                                  GL_UNSIGNED_BYTE,
//                                  imAndTexFrame[i].first.data);
//                 }
//             }

//             UpdateModel();

//             glEnable(GL_TEXTURE_2D);

//             glBegin(GL_TRIANGLES);
//             glColor3f(1.0,1.0,1.0);

//             for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {

//                 dlovi::Matrix point0 = GetPoints()[(*it)(0)];
//                 dlovi::Matrix point1 = GetPoints()[(*it)(1)];
//                 dlovi::Matrix point2 = GetPoints()[(*it)(2)];

//                 dlovi::Matrix edge10 = point1 - point0;
//                 dlovi::Matrix edge20 = point2 - point0;

//                 dlovi::Matrix normal = edge20.cross(edge10);
//                 normal = normal / normal.norm();

//                 glNormal3d(normal(0), normal(1), normal(2));

//                 vector<double> dotProducts;
//                 vector<int> indexTex;
//                 for (int i = 0; i < numKFs; i++){
//                     cv::Mat texOrient = imAndTexFrame[i].second.GetOrientation();
//                     dlovi::Matrix orientation(3,1);
//                     orientation(0) = texOrient.at<float>(0);
//                     orientation(1) = texOrient.at<float>(1);
//                     orientation(2) = texOrient.at<float>(2);

//                     dotProducts.push_back(normal.dot(orientation));
//                     indexTex.push_back(i);
//                 }

//                 sort( begin(indexTex), end(indexTex),
//                       [&](int i1, int i2) { return dotProducts[i1] > dotProducts[i2]; } );

//                 for (int i = 0; i < numKFs; i++){
//                     int indexCurr = indexTex[i];

//                     TextureFrame tex = imAndTexFrame[indexCurr].second;
//                     vector<float> uv0 = tex.GetTexCoordinate(point0(0),point0(1),point0(2),imSize);
//                     vector<float> uv1 = tex.GetTexCoordinate(point1(0),point1(1),point1(2),imSize);
//                     vector<float> uv2 = tex.GetTexCoordinate(point2(0),point2(1),point2(2),imSize);

//                     if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {

//                         // TODO: not the right way to do multi-texturing
// //                        glDisable(GL_TEXTURE_2D);
// //                        glEnable(GL_TEXTURE_2D);
// //                        glBindTexture(GL_TEXTURE_2D, frameTex[indexCurr]);

//                         glTexCoord2f(uv0[0], uv0[1]);
//                         glVertex3d(point0(0), point0(1), point0(2));

//                         glTexCoord2f(uv1[0], uv1[1]);
//                         glVertex3d(point1(0), point1(1), point1(2));

//                         glTexCoord2f(uv2[0], uv2[1]);
//                         glVertex3d(point2(0), point2(1), point2(2));

//                         break;
//                     }
//                 }
//             }
//             glEnd();

//             glDisable(GL_TEXTURE_2D);
//         }
//     }

    void ModelDrawer::DrawModelPoints()
    {
        UpdateModel();

        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0.5, 0.5, 0.5);
        for (size_t i = 0; i < GetPoints().size(); i++) {
            glVertex3d(GetPoints()[i](0), GetPoints()[i](1), GetPoints()[i](2));
        }
        glEnd();
    }

    void ModelDrawer::DrawTriangles(pangolin::OpenGlMatrix &Twc)
    {
        UpdateModel();

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        glPopMatrix();

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        glShadeModel(GL_FLAT);

        GLfloat material_diffuse[] = {0.2, 0.5, 0.8, 1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_diffuse);

        glBegin(GL_TRIANGLES);
        glColor3f(1.0,1.0,1.0);

        for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {

            dlovi::Matrix point0 = GetPoints()[(*it)(0)];
            dlovi::Matrix point1 = GetPoints()[(*it)(1)];
            dlovi::Matrix point2 = GetPoints()[(*it)(2)];

            dlovi::Matrix edge10 = point1 - point0;
            dlovi::Matrix edge20 = point2 - point0;

            dlovi::Matrix normal = edge20.cross(edge10);
            normal = normal / normal.norm();

            glNormal3d(normal(0), normal(1), normal(2));

            glVertex3d(point0(0), point0(1), point0(2));
            glVertex3d(point1(0), point1(1), point1(2));
            glVertex3d(point2(0), point2(1), point2(2));

        }
        glEnd();

        glDisable(GL_LIGHTING);

    }

    // void ModelDrawer::DrawFrame(bool bRGB)
    // {
    //     // select the last frame
    //     int numKFs = 1;
    //     vector<pair<cv::Mat,TextureFrame>> imAndTexFrame = mpModeler->GetTextures(numKFs);

    //     if (imAndTexFrame.size() >= numKFs) {
    //         glColor3f(1.0,1.0,1.0);

    //         if (imAndTexFrame[0].first.empty()){
    //             std::cerr << "ERROR: empty frame image" << endl;
    //             return;
    //         }
    //         cv::Size imSize = imAndTexFrame[0].first.size();

    //         if(bRGB) {
    //             pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_BGR,
    //                                              GL_UNSIGNED_BYTE);
    //             imageTexture.Upload(imAndTexFrame[0].first.data, GL_BGR, GL_UNSIGNED_BYTE);
    //             imageTexture.RenderToViewportFlipY();
    //         } else {
    //             pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_RGB,
    //                                              GL_UNSIGNED_BYTE);
    //             imageTexture.Upload(imAndTexFrame[0].first.data, GL_RGB, GL_UNSIGNED_BYTE);
    //             imageTexture.RenderToViewportFlipY();
    //         }

    //     }
    // }
    void ModelDrawer::DrawFrame(bool bRGB, vector<pair<cv::Mat,TextureFrame>> imAndTexFrame)
    {
        // Get the textures from the Modeler
        int numKFs = imAndTexFrame.size();

        // Set up the grid layout parameters
        const int cols = 2; // Number of columns for the grid
        const int rows = (numKFs + cols - 1) / cols; // Calculate number of rows
        const int textureWidth = 512; // Desired width for each texture
        const int textureHeight = 512; // Desired height for each texture

        for (int i = 0; i < numKFs; ++i) {
            // Get the size of the current texture
            cv::Size imSize = imAndTexFrame[i].first.size();

            // Create a texture object
            pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, bRGB ? GL_BGR : GL_RGB, GL_UNSIGNED_BYTE);

            // Upload the texture data
            imageTexture.Upload(imAndTexFrame[i].first.data, bRGB ? GL_BGR : GL_RGB, GL_UNSIGNED_BYTE);

            // Calculate the position for the current texture
            int xPos = (i % cols) * textureWidth; // X position in grid
            int yPos = (i / cols) * textureHeight; // Y position in grid

            // Set up the viewport to render the texture
            pangolin::View& viewport = pangolin::CreateDisplay()
                .SetBounds(yPos / (float)(rows * textureHeight), (yPos + textureHeight) / (float)(rows * textureHeight), 
                        xPos / (float)(cols * textureWidth), (xPos + textureWidth) / (float)(cols * textureWidth));
            
            viewport.Activate();

            // Render the texture
            imageTexture.RenderToViewportFlipY();
        }
    }

    cv::Mat ModelDrawer::DrawLines()
    {
        cv::Mat im = mpModeler->GetImageWithLines();
        return im;
    }


    void ModelDrawer::UpdateModel()
    {
        if(mbModelUpdateRequested && ! mbModelUpdateDone)
            return;

        if(mbModelUpdateRequested && mbModelUpdateDone){
            mModel = mUpdatedModel;
            mbModelUpdateRequested = false;
            return;
        }

        mbModelUpdateDone = false;
        mbModelUpdateRequested = true; // implicitly signals SurfaceInferer thread which is polling
    }

    void ModelDrawer::SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris)
    {
        mUpdatedModel.first = modelPoints;
        mUpdatedModel.second = modelTris;
    }

    vector<dlovi::Matrix> & ModelDrawer::GetPoints()
    {
        return mModel.first;
    }

    list<dlovi::Matrix> & ModelDrawer::GetTris()
    {
        return mModel.second;
    }

    void ModelDrawer::MarkUpdateDone()
    {
        mbModelUpdateDone = true;
    }

    bool ModelDrawer::UpdateRequested()
    {
        return mbModelUpdateRequested;
    }

    bool ModelDrawer::UpdateDone()
    {
        return mbModelUpdateDone;
    }

    void ModelDrawer::SetModeler(Modeler* pModeler)
    {
        mpModeler = pModeler;
    }

} //namespace ORB_SLAM
