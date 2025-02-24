namespace Models
{
    public class GoodsItem
    {
        public string Id { get; private set; }
        public string Name { get; private set; }
        public string ImageSource { get; set; }

        public GoodsItem(string name, string imageSource)
        {
            Id = Guid.NewGuid().ToString();
            Name = name;
            ImageSource = imageSource;
        }
    }
}
