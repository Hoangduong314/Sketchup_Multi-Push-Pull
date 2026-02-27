module YourName
  module MultiPushPull
    class MultiPushPullTool

      def create_geometry_individual(ents)
        affected_faces = []
        @selection.each do |face|
          offset_vector = face.normal.clone
          offset_vector.length = @distance.abs
          offset_vector.reverse! if @distance < 0

          face.edges.each do |edge|
            v1 = edge.reversed_in?(face) ? edge.end : edge.start
            v2 = edge.reversed_in?(face) ? edge.start : edge.end
            
            pt1 = v1.position
            pt2 = v2.position
            new_pt1 = pt1.offset(offset_vector)
            new_pt2 = pt2.offset(offset_vector)
            
            edge_vec = pt1.vector_to(pt2)
            outward_norm = edge_vec.cross(face.normal)
            outward_norm.reverse! if @distance < 0
            
            created_faces = create_side_faces(ents, pt1, pt2, new_pt2, new_pt1, outward_norm)
            affected_faces.concat(created_faces)
          end

          new_pts = face.vertices.map { |v| v.position.offset(offset_vector) }
          target_norm = (@distance < 0 && @keep_original) ? face.normal.reverse : face.normal
          caps = create_cap_faces(ents, face, new_pts, target_norm)
          affected_faces.concat(caps)
        end

        unless @keep_original
          faces_to_erase = @selection.select(&:valid?)
          ents.erase_entities(faces_to_erase) if faces_to_erase.any?
        end

        return affected_faces
      end

      def draw_individual(view, edge_color, tr)
        @selection.each do |face|
          offset_vector = face.normal.clone
          offset_vector.length = @distance.abs
          offset_vector.reverse! if @distance < 0

          original_pts = face.vertices.map { |v| v.position.transform(tr) }
          new_pts = face.vertices.map { |v| v.position.offset(offset_vector).transform(tr) }

          view.line_width = 2
          view.drawing_color = edge_color
          view.draw(GL_LINE_LOOP, new_pts)

          original_pts.each_with_index do |pt, i|
            next_pt = original_pts[(i + 1) % original_pts.length]
            new_pt = new_pts[i]
            new_next_pt = new_pts[(i + 1) % new_pts.length]

            view.line_width = 1
            view.drawing_color = edge_color
            view.draw(GL_LINE_LOOP, [pt, next_pt, new_next_pt, new_pt])
          end
        end
      end

    end
  end
end