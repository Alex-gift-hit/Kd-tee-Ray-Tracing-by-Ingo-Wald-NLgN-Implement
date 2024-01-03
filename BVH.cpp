#include <algorithm>
#include <cassert>
#include "BVH.hpp"
BVHAccel::BVHAccel(std::vector<Object*> p, Bounds3 bounds, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if(splitMethod == SplitMethod::NAIVE) root = recursiveBuild(primitives);
    if(splitMethod == SplitMethod::SAH) {
        generateEvents(primitives);
        rootSAH = recursiveSAHBuild(primitives, this->Events ,bounds, 0);
    }

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
    int a;
    std::cin>>a;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    std::cout<<"In BVH Build!\n"<<std::endl;
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

// add
// There are two types of kd-tree based on stop spliting rule.
// If we want to change it, just comment out one of them.
SAHBuildNode* BVHAccel::recursiveSAHBuild(std::vector<Object*> objects, EventsXYZ Events, Bounds3 bounds, int kdDepth)
{
    
    
    SAHBuildNode* node = new SAHBuildNode();
    std::cout<<objects.size()<<std::endl;
    node->bounds = bounds;

    // 1. Limit stop
    if(objects.size() <= 4 || kdDepth>=16 ) {
        node->isleaf = true;
        if(objects.size() == 0) return node;
        for(auto obj : objects) node->object.emplace_back(obj);
        return node;
    }

    Plane plane = findPlane(objects.size(), bounds, Events);
    std::cout<<"Node pMin "<<node->bounds.pMin<<std::endl;
    std::cout<<"Node pMax "<<node->bounds.pMax<<std::endl;

    std::cout<<"Plane Axis "<<plane.splitAxis<<" "<< "Plane Dis "<<plane.splitDis<<" "<<std::endl;
    std::cout<<"kdDepth "<<kdDepth<<std::endl;

    // 2. Auto stop
    /*
    if(plane.cost > 1.5 * objects.size()){
        node->isleaf = true;
        if(objects.size() == 0) return node;
        for(auto obj : objects) node->object.emplace_back(obj);
        return node;
    }*/

    //SplitEAT ClassifyLeftRightBoth(std::vector<Object*> objs, EventsXYZ& Events, Plane p, Plane::PlaneSide ps, std::vector<Bounds3> bounds);
    node->splitAxis =  plane.splitAxis;
    node->splitDis = plane.splitDis;
    auto b = node->getBox();
    SplitEAT EAT = ClassifyLeftRightBoth(objects, Events, plane, b); 
    
    std::cout<<"EAT.objLeft.size: "<<EAT.objLeft.size()<<" EAT.distEventLeft.x.size() "<< EAT.distEventLeft.x.size()<<std::endl;
    node->left = recursiveSAHBuild(EAT.objLeft, EAT.distEventLeft, b[0], ++kdDepth);

    std::cout<<"EAT.objRight.size: "<<EAT.objRight.size()<<" EAT.distEventRight.x.size() "<< EAT.distEventRight.x.size()<<std::endl;
    node->right = recursiveSAHBuild(EAT.objRight, EAT.distEventRight, b[1], ++kdDepth);

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (splitMethod == SplitMethod::SAH){
        
        isect = BVHAccel::getIntersection(rootSAH, ray);
    }

    if (splitMethod == SplitMethod::NAIVE ){
        isect = BVHAccel::getIntersection(root, ray);
    }
    
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    
    std::cout<<"BVHACC getItersection"<<std::endl;
    if(node->bounds.IntersectP(ray,ray.direction_inv, {0,0,0}) ){
        if(node->right == nullptr && node->left ==  nullptr) {
            return node->object->getIntersection(ray);
        }else{
            
            auto l = getIntersection(node->left, ray);
            auto r = getIntersection(node->right, ray); 
            return l.distance < r.distance? l:r;
        }

    }else{
        std::cout<<"BVH not Hit!"<<std::endl;
        return Intersection();
        
    }
}

Intersection BVHAccel::getIntersection(SAHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    
    //std::cout<<"SAHACC getItersection"<<std::endl;
    if(node->bounds.IntersectP(ray,ray.direction_inv, {0,0,0}) ){
        if(node->isleaf) {
            if(node->object.size() == 0) {
                return Intersection();
            }else{

                Intersection IS;
                /*for(auto obj: node->object){
                    auto ret = obj->getIntersection(ray);
                    if(ret.distance < IS.distance) {IS = ret;};
                }*/
                for(auto obj : node->object){
                    if(obj->getBounds().IntersectP(ray,ray.direction_inv, {0,0,0})){
                        auto ret = obj->getIntersection(ray);
                        if(ret.distance < IS.distance) {IS = ret;};
                    }
                }
                return IS;
            }
            
        }else{
            
            auto l = getIntersection(node->left, ray);
            auto r = getIntersection(node->right, ray); 
            return l.distance < r.distance? l:r;
        }

    }else{
        std::cout<<"SAH not Hit!"<<std::endl;
        return Intersection();
        
    }
}

// std::vector<EventNode> Events;// add
// From Ingo Wald: kd-tree NlogN page 6, algorithm 4.

bool BVHAccel::generateEvents(std::vector<Object*> objects){
    if(objects.size() == 0) return false;
    for(auto obj : objects){
        for(int k=0; k < 3; k++){
            EventNode temp;
            Bounds3 bound = obj->getBounds();
            if(bound.pMin[k] == bound.pMax[k] ){// planar
                temp.plane.splitAxis = k;
                temp.plane.splitDis = bound.pMin[k];
                temp.eventType = 1;
                temp.object = obj;
                Events[k].emplace_back(temp);
            }else{
                temp.plane.splitAxis = k;
                temp.object = obj;
                temp.plane.splitDis = bound.pMin[k];
                temp.eventType = 2;// start
                Events[k].emplace_back(temp);

                temp.plane.splitDis = bound.pMax[k];
                temp.eventType = 0;// end
                Events[k].emplace_back(temp);
            }

        }
    }

    for(int k=0; k<3; k++){
        std::sort(Events[k].begin(), Events[k].end(), comp);
    }
 

    return true;

}

Plane BVHAccel::findPlane(int triNum, Bounds3 bound, EventsXYZ& Events){
    NumOfTri numOfTri;
    for(int k=0; k<3; k++){
        numOfTri.nLeft[k]=0;
        numOfTri.nPlane[k]=0;
        numOfTri.nRight[k]=triNum;
    }

    float cost = std::numeric_limits<float>::infinity();
    Plane pResult(0, (bound.pMax[0] + bound.pMin[0]) / 2);
    Plane::PlaneSide pSide;

    for(int k = 0; k<3; k++){
        for(int i=0; i<Events[k].size(); i++){
            Plane p(k, Events[k][i].plane.splitDis);
            int pPlus = 0, pMinus=0, pVer = 0;

            while(i < Events[k].size() && p.splitDis == Events[k][i].plane.splitDis && Events.x[i].eventType == 0){
                i++;
                pMinus++;
            }

            while(i < Events[k].size() && p.splitDis == Events[k][i].plane.splitDis && Events.x[i].eventType == 1){
                i++;
                pVer++;
            }

            while(i < Events[k].size() && p.splitDis == Events[k][i].plane.splitDis && Events.x[i].eventType == 2){
                i++;
                pPlus++;
            }
            numOfTri.nPlane[k] = pVer;
            numOfTri.nRight[k] -= (pVer+pMinus);

            auto [costTemp, p_side] = SAH(bound, p, numOfTri.nLeft[k], numOfTri.nRight[k], numOfTri.nPlane[k]);
            if(costTemp < cost){
                cost = costTemp;
                pResult = p;
                pSide = p_side;
            }

            numOfTri.nLeft[k] += (pVer+pPlus);
            numOfTri.nPlane[k] = 0;
        }
    }
    pResult.ps = pSide;
    return pResult; 
    
}

std::pair<float, Plane::PlaneSide> BVHAccel::SAH(Bounds3 bounds, Plane p, int Nl, int Nr, int Np){

    // split Box
    std::vector<Bounds3> b;
    Vector3f bLMax = bounds.pMax;
    //std::cout<<bLMax<<std::endl;
    bLMax[p.splitAxis] = p.splitDis;
    //std::cout<<bLMax<<std::endl;
    Bounds3 B1{bounds.pMin, bLMax};
    b.push_back(B1);
    Vector3f bRMin = bounds.pMin;
    bRMin[p.splitAxis] = p.splitDis;
    Bounds3 B2{bRMin, bounds.pMax};
    b.push_back(B2);

    double Pl = b[0].SurfaceArea() / bounds.SurfaceArea();
    double Pr = b[1].SurfaceArea() / bounds.SurfaceArea();

    float lambda = 1;
    if(Nl == 0 || Nr == 0){
        lambda = 0.8;
        if(bounds.pMin[p.splitAxis] == p.splitDis || bounds.pMax[p.splitAxis] == p.splitAxis) lambda*=10000;
    }

    float costLeft  = lambda * (1.0 + 1.5 * (Pl * (Nl + Np) + Pr * Nr));
    float costRight = lambda * (1.0 + 1.5 * (Pl * Nl + Pr * (Nr + Np)));

    if(costLeft < costRight){
        return {costLeft, Plane::PlaneSide::LEFT};
    }else{
        return {costRight, Plane::PlaneSide::RIGHT};
    }
}


SplitEAT BVHAccel::ClassifyLeftRightBoth(std::vector<Object*> objs, EventsXYZ& Events, Plane p, std::vector<Bounds3> bounds){
    for(auto obj : objs){
        obj->setSide(Object::Side::BOTH);
    }

    for(int k = 0; k< 3; k++){
        for(auto& event : Events[k]){
            if(event.eventType == 0 && event.plane.splitDis <= p.splitDis && k == p.splitAxis){
                event.object->setSide(Object::Side::LEFT);
            }else if(event.eventType == 2 && event.plane.splitDis >= p.splitDis && k == p.splitAxis){
                event.object->setSide(Object::Side::RIGHT);
            }else if(event.eventType == 1 && event.plane.splitDis == p.splitDis && k == p.splitAxis){
                if(p.ps == Plane::PlaneSide::LEFT){
                    event.object->setSide(Object::Side::LEFT);
                }else{
                    event.object->setSide(Object::Side::RIGHT);
                }
            }
        }
    }
    
    EventsXYZ eventsLeft;
    EventsXYZ eventsRight;

    for(int k = 0; k<3; k++){
        for(auto& event : Events[k]){
        switch(event.object->getSide()){
            case Object::Side::LEFT :
            {
                eventsLeft[k].emplace_back(event);
                break;
            }
            case Object::Side::RIGHT:
            {
                eventsRight[k].emplace_back(event);
                break; 
            }
            default:
                break;
        }
    }
    }
    
    Bounds3 clipBox;
    EventsXYZ eventsNewLeft;
    EventsXYZ eventsNewRight;
    std::vector<Object*> objLeft;
    std::vector<Object*> objRight;

    for(auto obj:objs){
        if(obj->getSide() == Object::Side::LEFT) objLeft.emplace_back(obj);
        if(obj->getSide() == Object::Side::RIGHT) objRight.emplace_back(obj);
        if(obj->getSide() == Object::Side::BOTH){
            objLeft.emplace_back(obj);
            objRight.emplace_back(obj);

            clipBox = clipBox.Clip(obj->getBounds(), bounds[0]);
            for(int k=0; k < 3; k++){
                EventNode temp;
                if(clipBox.pMin[k] == clipBox.pMax[k] ){// planar
                    temp.plane.splitAxis = k;
                    temp.plane.splitDis =clipBox.pMin[k];
                    temp.eventType = 1;
                    temp.object = obj;
                    eventsNewLeft[k].emplace_back(temp);
                }else{
                    temp.plane.splitAxis = k;
                    temp.object = obj;
                    temp.plane.splitDis = clipBox.pMin[k];
                    temp.eventType = 2;// start
                    eventsNewLeft[k].emplace_back(temp);

                    temp.plane.splitDis = clipBox.pMax[k];
                    temp.eventType = 0;// end
                    eventsNewLeft[k].emplace_back(temp);
                }

            }
            //genrate left
            clipBox = clipBox.Clip(obj->getBounds(), bounds[1]);
            //genrate right
            // todo
            for(int k=0; k < 3; k++){
                EventNode temp;
                if(clipBox.pMin[k] == clipBox.pMax[k] ){// planar
                    temp.plane.splitAxis = k;
                    temp.plane.splitDis =clipBox.pMin[k];
                    temp.eventType = 1;
                    temp.object = obj;
                    eventsNewRight[k].emplace_back(temp);
                }else{
                    temp.plane.splitAxis = k;
                    temp.object = obj;
                    temp.plane.splitDis = clipBox.pMin[k];
                    temp.eventType = 2;// start
                    eventsNewRight[k].emplace_back(temp);

                    temp.plane.splitDis = clipBox.pMax[k];
                    temp.eventType = 0;// end
                    eventsNewRight[k].emplace_back(temp);
                }

            }
        }
    }

    for(int k=0; k<3; k++){
        std::sort(eventsNewLeft[k].begin(), eventsNewLeft[k].end(), comp);

        std::sort(eventsNewRight[k].begin(), eventsNewRight[k].end(), comp);
    }

    EventsXYZ distEventLeft;
    EventsXYZ distEventRight;

    for(int k = 0; k < 3; k++){
 
        distEventLeft[k].resize(eventsLeft[k].size() + eventsNewLeft[k].size());
        std::merge(eventsLeft[k].begin(), eventsLeft[k].end(), eventsNewLeft[k].begin(), eventsNewLeft[k].end(), distEventLeft[k].begin(), comp);

        distEventRight[k].resize(eventsRight[k].size() + eventsNewRight[k].size());
        std::merge(eventsRight[k].begin(), eventsRight[k].end(), eventsNewRight[k].begin(), eventsNewRight[k].end(), distEventRight[k].begin(), comp);
    }


    SplitEAT result;

    // event 
    result.distEventLeft = distEventLeft;
    result.distEventRight = distEventRight;


    // primitive
    result.objLeft = objLeft;
    result.objRight = objRight;

    return result;
}
